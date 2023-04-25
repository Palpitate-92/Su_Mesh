// #pragma once
#ifndef _SUMESH_H_
// 如果一个文件包含了此头文件多次，使用这种方法，即在第一次编译时没有定义xxx的宏，执行了下面的所有，第二次再遇到编译此文件时xxx已经被定义，就不会再编译
//  使用条件编译可以避免重复编译
//  需要注意的是要使用#ifndef语句
#define _SUMESH_H_

#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <math.h>
#include <algorithm>

#define REAL double /* float or double */

extern const int DIM = 2; // 声明维度DIM为常值
// extern const double General_spac = 0.08;        // 网格单元边一般理想长度
// extern const double HighDensity_spac = 0.025;   // 网格单元边高密度情形理想长度
// extern const double L_HighDensity_spac = 0.07;  // 网格单元边低高密度情形理想长度
// extern const double H_HighDensity_spac = 0.005; // 网格单元边超高密度情形理想长度
extern const double Pi = 3.14159265358979;                  // 声明圆周率
extern const double angle_ideal = 20;                       // 网格单元理想角度，只要一个角的度数低于该度数，则不予生成，单位度
extern const double Minimum_Shape_Quality_Swap = 0.6;       // 网格单元质量的极值，质量小于该值的网格单元与相邻单元运行边交换
extern const double Minimum_Shape_Quality_Smoothing = 0.85; // 网格单元质量的极值，质量小于该值的网格单元运行节点光顺
extern const double Delta = 1e-8;                           // 作为函数求导时的Delta使用，和一些判断时的误差消除
extern const double Min_step = 1e-6;                        // 最小步长，基于优化的光顺内使用
extern const int Max_iter = 7;                              // 最大迭代次数，基于优化的光顺内使用
extern const double Min_imp = 1e-8;                         // 最小差距值，基于优化的光顺使用，比较两次迭代后，优化域目标函数的值变换，小于该值则退出循环，接受坐标
extern const double Min_gradient = 1e-8;                    // 最小梯度值，梯度向量的模小于该值时代表达到极值点处，停止迭代
extern const double c1 = 1e-3;                              // Armijo准则的常数
extern const double c2 = 0.9;                               // Wolfe准则的常数

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
    for (int i = 0; i < DIM; i++)
    {
      pos[i] = 0; // 节点坐标应该初始化为零
    }
    elem = -1;
    spac = -1;
  }
  NODE(double pos1, double pos2, int elem_tp, double spac_tp)
  {
    pos[0] = pos1;
    pos[1] = pos2;
    elem = elem_tp;
    spac = spac_tp;
  }
  NODE(const NODE &node) // 拷贝构造函数，支持声明NODE类时赋值
  {
    for (int i = 0; i < DIM; i++)
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
  NODE operator+(const NODE &node) const               // 重载+运算符，支持函数导数运算
  {
    NODE node_tp;
    node_tp.pos[0] = pos[0] + node.pos[0];
    node_tp.pos[1] = pos[1] + node.pos[1];
    node_tp.elem = elem;
    node_tp.spac = spac;
    return node_tp;
  }
  NODE operator-(const NODE &node) const // 重载-运算符，支持函数导数运算
  {
    NODE node_tp;
    node_tp.pos[0] = pos[0] - node.pos[0];
    node_tp.pos[1] = pos[1] - node.pos[1];
    node_tp.elem = elem;
    node_tp.spac = spac;
    return node_tp;
  }
  NODE operator+(const double value[2][1]) const // 重载-运算符，支持节点类与普通数组的运算
  {
    NODE node_tp;
    node_tp.pos[0] = pos[0] + value[0][0];
    node_tp.pos[1] = pos[1] + value[1][0];
    node_tp.elem = elem;
    node_tp.spac = spac;
    return node_tp;
  }
  ~NODE() {} // 析构函数

private:
  double pos[DIM]; // 节点几何坐标
  int elem;        // 当前网格中包含该节点的任意单元标号
  double spac;     // 对应节点理想单元尺寸值
};

// NODE类初始化
void InitNode(NODE *node_init)
{
  for (int i = 0; i < DIM; i++)
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

extern const NODE node_XDelta{Delta, 0, -1, -1}; // 声明一个NODE类，作为函数求导时X方向的Delta使用
extern const NODE node_YDelta{0, Delta, -1, -1}; // 声明一个NODE类，作为函数求导时Y方向的Delta使用

// 一些函数的提前声明，便于使用
REAL incircle(REAL *pa, REAL *pb, REAL *pc, REAL *pd);
REAL orient2d(REAL *pa, REAL *pb, REAL *pc);

/**
 * ? 下面是各种功能函数
 */

// 声明两点间距离计算算法
double getDist(double A[], double B[])
{
  return double(sqrt(pow(A[0] - B[0], 2) + pow(A[1] - B[1], 2)));
}

// 给定三角形三个顶点位置，返回三角形面积，顶点是逆时针方向的。如果三角形在我们的过程中重新定向，顶点顺时针方向排列，继续使用将产生一个负值区域，发出重定向的信号。
double Triangle_area(NODE node_tp1, NODE node_tp2, NODE node_tp3)
{
  double area = (node_tp2.get_pos(0) - node_tp1.get_pos(0)) * (node_tp3.get_pos(1) - node_tp1.get_pos(1)) -
                (node_tp3.get_pos(0) - node_tp1.get_pos(0)) * (node_tp2.get_pos(1) - node_tp1.get_pos(1));
  return area / 2;
}

// 给定三角形三个顶点位置，返回以node_tp3为顶角的角度数
double get_angle(NODE node_tp1, NODE node_tp2, NODE node_tp3)
{
  double theta = atan2(node_tp1.get_pos(0) - node_tp3.get_pos(0), node_tp1.get_pos(1) - node_tp3.get_pos(1)) -
                 atan2(node_tp2.get_pos(0) - node_tp3.get_pos(0), node_tp2.get_pos(1) - node_tp3.get_pos(1));
  if (theta > Pi)
    theta -= 2 * Pi;
  if (theta < -Pi)
    theta += 2 * Pi;
  theta = abs(theta * 180.0 / Pi);
  return theta;
}

// 生成初始Delaunay三角化四边形边框，并插入四个顶角节点
void Gen_IniDeFrame(std::vector<NODE> *node, int *node_num, int DeFrame_numPos[])
{
  double Ini_framePos[] = {0, 0, 0, 0}; // 定义输入图形初始全包围方形边框，分别为：x_min,x_max,y_min,y,max
  for (std::vector<NODE>::iterator node_iter = node->begin(); node_iter != node->end(); ++node_iter)
  {
    if (Ini_framePos[0] > node_iter->get_pos(0))
      Ini_framePos[0] = node_iter->get_pos(0);
    if (Ini_framePos[1] < node_iter->get_pos(0))
      Ini_framePos[1] = node_iter->get_pos(0);
    if (Ini_framePos[2] > node_iter->get_pos(1))
      Ini_framePos[2] = node_iter->get_pos(1);
    if (Ini_framePos[3] < node_iter->get_pos(1))
      Ini_framePos[3] = node_iter->get_pos(1);
  }
  double Ini_DeFrame[] = {0, 0, 0, 0}; // 定义初始Delaunay三角化四边形边框，分别为：x_min,x_max,y_min,y,max
  // 如果输入图形初始方形边框是正方形，则用各自边来赋值
  if ((Ini_framePos[1] - Ini_framePos[0]) == (Ini_framePos[3] - Ini_framePos[2]))
  {
    Ini_DeFrame[0] = Ini_framePos[0] - (Ini_framePos[1] - Ini_framePos[0]) / 2;
    Ini_DeFrame[1] = Ini_framePos[1] + (Ini_framePos[1] - Ini_framePos[0]) / 2;
    Ini_DeFrame[2] = Ini_framePos[2] - (Ini_framePos[3] - Ini_framePos[2]) / 2;
    Ini_DeFrame[3] = Ini_framePos[3] + (Ini_framePos[3] - Ini_framePos[2]) / 2;
  }
  // 如果输入图形初始方形边框不是正方形，是长方形，则用最长的边来赋值
  else
  {
    int tp = std::max((Ini_framePos[1] - Ini_framePos[0]), (Ini_framePos[3] - Ini_framePos[2]));
    Ini_DeFrame[0] = Ini_framePos[0] - tp / 2;
    Ini_DeFrame[1] = Ini_framePos[1] + tp / 2;
    Ini_DeFrame[2] = Ini_framePos[2] - tp / 2;
    Ini_DeFrame[3] = Ini_framePos[3] + tp / 2;
  }
  NODE node_tp;
  // 四边形边框有四个顶点
  node_tp.tr_pos(0, Ini_DeFrame[0]);
  node_tp.tr_pos(1, Ini_DeFrame[2]);
  node->push_back(node_tp);
  DeFrame_numPos[0] = (*node_num)++;
  InitNode(&node_tp);
  node_tp.tr_pos(0, Ini_DeFrame[1]);
  node_tp.tr_pos(1, Ini_DeFrame[2]);
  node->push_back(node_tp);
  DeFrame_numPos[1] = (*node_num)++;
  InitNode(&node_tp);
  node_tp.tr_pos(0, Ini_DeFrame[1]);
  node_tp.tr_pos(1, Ini_DeFrame[3]);
  node->push_back(node_tp);
  DeFrame_numPos[2] = (*node_num)++;
  InitNode(&node_tp);
  node_tp.tr_pos(0, Ini_DeFrame[0]);
  node_tp.tr_pos(1, Ini_DeFrame[3]);
  node->push_back(node_tp);
  DeFrame_numPos[3] = (*node_num)++;
  return;
}

// 生成初始Delaunay三角化
void Gen_IniDeDelaunay(std::vector<ELEM> *elem, int *elem_num, std::vector<NODE> *node, int DeFrame_numPos[])
{
  ELEM elem_tp;
  for (int i = 0; i < DIM + 1; i++)
  {
    elem_tp.tr_form(i, DeFrame_numPos[i]);
    node->at(DeFrame_numPos[i]).tr_elem(*elem_num); // 改变当前网格中包含该节点的任意单元标号
  }
  elem_tp.tr_neig(1, 1);
  elem->push_back(elem_tp);
  *elem_num += 1;
  InitElem(&elem_tp);
  elem_tp.tr_form(0, DeFrame_numPos[0]);
  node->at(DeFrame_numPos[0]).tr_elem(*elem_num);
  elem_tp.tr_form(1, DeFrame_numPos[2]);
  node->at(DeFrame_numPos[2]).tr_elem(*elem_num);
  elem_tp.tr_form(2, DeFrame_numPos[3]);
  node->at(DeFrame_numPos[3]).tr_elem(*elem_num);
  elem_tp.tr_neig(2, 0);
  elem->push_back(elem_tp);
  *elem_num += 1;
  return;
}

// 查找三角形外接圆包含待插入点的网格单元
int FindElemIncludeNode(std::vector<ELEM> *elem, std::vector<NODE> *node, NODE node_Insert, int elemNum_IncludeNodeInitial)
{
  std::vector<int> elemNum_wait;                      // 创建一个容器，用来储存待判断的网格单元
  std::vector<int> elemNum_succ;                      // 创建一个容器，用来储存判断过的单元
  elemNum_wait.push_back(elemNum_IncludeNodeInitial); // 压入初始网格单元
  while (!elemNum_wait.empty())
  {
    int elemNum_tp = elemNum_wait.front(); // 取出elemNum_wait中的第一个元素并擦除
    elemNum_wait.erase(elemNum_wait.begin());
    double node_num[4][2];     // 定义一个数组，储存网格单元节点与待插入点，便于后面数组值的传递
    double center_triangle[2]; // 定义一个数组，储存三角形重心
    for (int i = 0; i < DIM + 1; i++)
    {
      node_num[i][0] = node->at(elem->at(elemNum_tp).get_form(i)).get_pos(0);
      node_num[i][1] = node->at(elem->at(elemNum_tp).get_form(i)).get_pos(1);
    }
    center_triangle[0] = (node_num[0][0] + node_num[1][0] + node_num[2][0]) / 3;
    center_triangle[1] = (node_num[0][1] + node_num[1][1] + node_num[2][1]) / 3;
    node_num[3][0] = node_Insert.get_pos(0);
    node_num[3][1] = node_Insert.get_pos(1);
    // 由于此处要使三角形三个顶点按逆时针顺序排列，利用三角形重心一定在三角形内部这一特点
    // 计算三角形重心在三角形外接圆的位置正负号，来判断怎样排序node_num能使三角形三个顶点按逆时针顺序排列
    if (incircle(node_num[0], node_num[1], node_num[2], center_triangle) > 0)
    {
      if (incircle(node_num[0], node_num[1], node_num[2], node_num[3]) >= 0) // 判断待插入点是否在三角形外接圆内部
        return elemNum_tp;
    }
    else
    {
      if (incircle(node_num[0], node_num[2], node_num[1], node_num[3]) >= 0) // 判断待插入点是否在三角形外接圆内部
        return elemNum_tp;
    }
    elemNum_succ.push_back(elemNum_tp);
    // 将该网格单元周围未被判断过的相邻单元压入elemNum_wait
    for (int i = 0; i < DIM + 1; i++)
      // 首先判断该单元是否存在于elemNum_wait，如不存在再往下判断
      // 在elemNum_succ内查找elemNum_tp对应网格单元相邻网格单元，如果搜到end()，代表没有找到，即该网格没被判断过
      if (std::find(elemNum_wait.begin(), elemNum_wait.end(), elem->at(elemNum_tp).get_neig(i)) == elemNum_wait.end())
        if (std::find(elemNum_succ.begin(), elemNum_succ.end(), elem->at(elemNum_tp).get_neig(i)) == elemNum_succ.end())
          if (elem->at(elemNum_tp).get_neig(i) != -1)
            elemNum_wait.push_back(elem->at(elemNum_tp).get_neig(i));
  }
  return -1;
}

// 返回该网格单元的外接圆是否包含该节点
bool CirumsInclude(ELEM elem_tp, std::vector<NODE> *node, NODE node_Insert)
{
  // 因为是二维下，直接看x,y坐标就行，并且一个网格单元有三个节点
  double point_Coord[3][2] = {{node->at(elem_tp.get_form(0)).get_pos(0), node->at(elem_tp.get_form(0)).get_pos(1)},
                              {node->at(elem_tp.get_form(1)).get_pos(0), node->at(elem_tp.get_form(1)).get_pos(1)},
                              {node->at(elem_tp.get_form(2)).get_pos(0), node->at(elem_tp.get_form(2)).get_pos(1)}};
  double center_triangle[2];                                                    // 定义一个数组，储存三角形重心
  double node_Insert_pos[2] = {node_Insert.get_pos(0), node_Insert.get_pos(1)}; // 定义一个数组，储存待插入点坐标
  center_triangle[0] = (point_Coord[0][0] + point_Coord[1][0] + point_Coord[2][0]) / 3;
  center_triangle[1] = (point_Coord[0][1] + point_Coord[1][1] + point_Coord[2][1]) / 3;
  // 由于此处要使三角形三个顶点按逆时针顺序排列，利用三角形重心一定在三角形内部这一特点
  // 计算三角形重心在三角形外接圆的位置正负号，来判断怎样排序node_num能使三角形三个顶点按逆时针顺序排列
  if (incircle(point_Coord[0], point_Coord[1], point_Coord[2], center_triangle) > 0)
  {
    if (incircle(point_Coord[0], point_Coord[1], point_Coord[2], node_Insert_pos) >= 0) // 判断待插入点是否在三角形外接圆内部
      return true;
  }
  else
  {
    if (incircle(point_Coord[0], point_Coord[2], point_Coord[1], node_Insert_pos) >= 0) // 判断待插入点是否在三角形外接圆内部
      return true;
  }
  return false;
}

// 判断某个单元是否包含某个节点编号，并返回该节点编号在该单元的form中位置，返回-1则代表不包含
int ElemIncludeNode(ELEM elem_tp, int nodeNum_tp)
{
  for (int i = 0; i < DIM + 1; i++)
    if (elem_tp.get_form(i) == nodeNum_tp)
      return i;
  return -1;
}

// 给定两个网格单元，利用两个网格单元的节点判断两个网格单元是否相邻
bool Elem_Adjacent(ELEM elem_tp1, ELEM elem_tp2)
{
  // 读入6个节点编号
  int arr[] = {elem_tp1.get_form(0), elem_tp1.get_form(1), elem_tp1.get_form(2),
               elem_tp2.get_form(0), elem_tp2.get_form(1), elem_tp2.get_form(2)};
  // 对着6个节点编号进行排序，便于判断相同节点编号的组数
  std::sort(arr, arr + 6);
  // 计数器，记录有几组相同的节点编号
  int cnt = 0;
  for (int i = 0; i < 5; i++)
  {
    if (arr[i + 1] == arr[i])
      cnt++;
  }
  if (cnt == 2)
    return true;
  else
    return false;
}

// 给定一个网格单元与该网格单元相邻的网格单元编号，返回指定单元编号在该单元的neig中位置，返回-1则代表输入有误，两个网格单元不相邻，适用于非改变elem容器时的判断
int AdjacentElem_pos(ELEM elem_tp, int elemNum_tp)
{
  for (int i = 0; i < DIM + 1; i++)
    if (elem_tp.get_neig(i) == elemNum_tp)
      return i;
  return -1;
}

// 给定一个网格单元和网格单元任意一个节点，返回该网格单元中该节点对应的单元边
EDGE Node_Opposite_Face(ELEM elem_tp, int node_tp)
{
  EDGE edge_tp;
  if (elem_tp.get_form(0) == node_tp)
  {
    edge_tp.tr_form(0, elem_tp.get_form(1));
    edge_tp.tr_form(1, elem_tp.get_form(2));
  }
  else if (elem_tp.get_form(1) == node_tp)
  {
    edge_tp.tr_form(0, elem_tp.get_form(0));
    edge_tp.tr_form(1, elem_tp.get_form(2));
  }
  else
  {
    edge_tp.tr_form(0, elem_tp.get_form(0));
    edge_tp.tr_form(1, elem_tp.get_form(1));
  }
  return edge_tp;
}

// 给定一个网格单元和网格单元的任意一条边，返回该网格单元边相对节点在elem.neig中的位置
int Face_Opposite_Node(ELEM elem_tp, EDGE edge_tp)
{
  // 由于任意一条边的两个节点总是按节点编号从小到大排序，所以可以直接判断
  if (elem_tp.get_form(0) != edge_tp.get_form(0))
    return 0;
  else if (elem_tp.get_form(1) != edge_tp.get_form(1))
    return 1;
  else
    return 2;
}

// 给定一个网格单元与跟该网格单元必定相邻的网格单元编号，返回该两个网格单元的相邻边
EDGE elem_AdjacentEdge(ELEM elem_tp, int elemNum_tp)
{
  EDGE edge_tp; // 声明一个变量，储存相邻边
  // 判断该网格单元编号所代表的网格单元在给定网格单元的neig中位置，来判断相邻边
  switch (AdjacentElem_pos(elem_tp, elemNum_tp))
  {
  case 0:
    edge_tp.tr_form(0, elem_tp.get_form(1));
    edge_tp.tr_form(1, elem_tp.get_form(2));
    break;
  case 1:
    edge_tp.tr_form(0, elem_tp.get_form(0));
    edge_tp.tr_form(1, elem_tp.get_form(2));
    break;
  case 2:
    edge_tp.tr_form(0, elem_tp.get_form(0));
    edge_tp.tr_form(1, elem_tp.get_form(1));
    break;
  default:
    break;
  }
  return edge_tp;
}

// 给定两个必定相邻的网格单元编号，返回他们相对节点形成的边
EDGE elem_OppositeEdge(std::vector<ELEM> *elem, int elemNum_tp_1, int elemNum_tp_2)
{
  EDGE edge_tp;
  edge_tp.tr_form(0, elem->at(elemNum_tp_1).get_form(AdjacentElem_pos(elem->at(elemNum_tp_1), elemNum_tp_2)));
  edge_tp.tr_form(1, elem->at(elemNum_tp_2).get_form(AdjacentElem_pos(elem->at(elemNum_tp_2), elemNum_tp_1)));
  edge_tp.Sort(); // 使两个节点编号从小到大排序
  return edge_tp;
}

// 给定三个必定相邻的网格单元编号，返回该三个网格单元皆包含的节点，返回-1代表出错
int ThreeElem_containsNode(std::vector<ELEM> *elem, int elemNum_tp_1, int elemNum_tp_2, int elemNum_tp_3)
{
  EDGE edge_tp;                                                  // 声明一个EDGE变量，临时储存相邻边
  edge_tp = elem_OppositeEdge(elem, elemNum_tp_1, elemNum_tp_2); // 获取任意两个网格单元的相邻边
  for (int i = 0; i < DIM; i++)
    if (ElemIncludeNode(elem->at(elemNum_tp_3), edge_tp.get_form(i)) != -1)
      return edge_tp.get_form(i);
  return -1;
}

// 给定一个网格单元编号，使该网格单元内所有节点的elem指向该网格单元
void Renew_NodeElem(std::vector<ELEM> *elem, std::vector<NODE> *node, int elemNum_tp)
{
  for (int i = 0; i < DIM + 1; i++)
    node->at(elem->at(elemNum_tp).get_form(i)).tr_elem(elemNum_tp);
  return;
}

// 空腔查找
void FindElemCav(std::vector<ELEM> *elem, int *elem_num, std::vector<NODE> *node, int *node_num,
                 std::vector<int> *elemNum_Cav, NODE node_Insert, int elemNum_IncludeNodeIns)
{
  std::vector<int> elemNum_wait;                  // 创建一个容器，用来储存待判断的单元
  std::vector<int> elemNum_succ;                  // 创建一个容器，用来储存判断过的单元
  elemNum_succ.push_back(elemNum_IncludeNodeIns); // 直接压入待插入节点所处的单元编号到elemNum_succ，不用判断
  for (int i = 0; i < DIM + 1; i++)               // 首先压入待插入节点所处的单元的相邻单元到elemNum_wait
    if (elem->at(elemNum_IncludeNodeIns).get_neig(i) != -1)
      elemNum_wait.push_back(elem->at(elemNum_IncludeNodeIns).get_neig(i));
  int elemNum_tp;
  while (!elemNum_wait.empty()) // 只要elemNum_wait内有元素就继续查找
  {
    elemNum_tp = elemNum_wait.front(); // 取出elemNum_wait中的第一个元素并擦除
    elemNum_wait.erase(elemNum_wait.begin());
    // 判断该网格单元外接圆是否包含待插入节点
    bool elem_judge = CirumsInclude(elem->at(elemNum_tp), node, node_Insert);
    elemNum_succ.push_back(elemNum_tp); // 判断后压入elemNum_succ
    // 如果包含，则将该网格压入空腔，并将未判断过的相邻网格单元压入elemNum_wait
    if (elem_judge)
    {
      elemNum_Cav->push_back(elemNum_tp); // 将该网格单元编号压入空腔
      // 将该网格单元周围未被判断过的相邻单元压入elemNum_wait
      for (int i = 0; i < DIM + 1; i++)
        // 首先判断该单元是否存在于elemNum_wait，如不存在再往下判断
        if (std::find(elemNum_wait.begin(), elemNum_wait.end(), elem->at(elemNum_tp).get_neig(i)) == elemNum_wait.end())
          // 在elemNum_succ内查找elemNum_tp对应网格单元相邻网格单元，如果搜到end()，代表没有找到，即该网格没被判断过
          if (std::find(elemNum_succ.begin(), elemNum_succ.end(), elem->at(elemNum_tp).get_neig(i)) == elemNum_succ.end())
            // 判断该网格相邻网格单元是否有效，-1为无效单元
            if (elem->at(elemNum_tp).get_neig(i) != -1)
              // 判断该单元相邻网格单元是否是初始Delaunay三角化方形边框的四个顶角节点与边界点形成的单元
              if (!elem->at(elem->at(elemNum_tp).get_neig(i)).get_DeFrame_Grid())
                elemNum_wait.push_back(elem->at(elemNum_tp).get_neig(i));
    }
  }
  return;
}

// 空腔初始化操作，在elem内中初始化空腔所包含的网格单元
void InitElemCav(std::vector<ELEM> *elem, std::vector<int> *elemNum_Cav)
{
  ELEM elem_tp;       // 创建一个网格单元，用于初始化
  InitElem(&elem_tp); // 初始化
  for (unsigned long long i = 0; i < elemNum_Cav->size(); ++i)
    elem->at(elemNum_Cav->at(i)) = elem_tp;
}

// 空腔修复
void RepairCavity(std::vector<ELEM> *elem, int *elem_num, std::vector<NODE> *node, int *node_num,
                  std::vector<int> *elemNum_Cav, NODE node_Insert)
{
  if (*elem_num == 0) // 如果问题域内无网格单元则直接返回
    return;
  return;
}

// 查找包含节点的ball（球），利用节点的elem与网格单元的相邻关系快速查找包含某个节点的所有网格单元，适用于一般情况下查找节点
void FindBall_fast(std::vector<ELEM> *elem, std::vector<NODE> *node, int nodeNum_wait, std::vector<int> *elemNum_IncludeNode)
{
  ELEM elem_tp;                                                      // 声明一个变量，用于临时储存待判断网格单元
  std::vector<int> elemNum_wait;                                     // 创建一个容器，用来储存待判断的网格单元
  std::vector<int> elemNum_succ;                                     // 创建一个容器，用来储存判断过的单元
  elemNum_IncludeNode->push_back(node->at(nodeNum_wait).get_elem()); // 压入该节点的elem值，即已知包含该节点的网格单元编号
  elemNum_wait.push_back(node->at(nodeNum_wait).get_elem());         // 压入该节点的elem值，即已知包含该节点的网格单元编号
  while (!elemNum_wait.empty())
  {
    int elemNum_tp = elemNum_wait.front(); // 取出elemNum_wait中的第一个元素并擦除
    elemNum_wait.erase(elemNum_wait.begin());
    InitElem(&elem_tp);
    elem_tp = elem->at(elemNum_tp);
    for (int i = 0; i < DIM + 1; i++) // 检索elemNum_tp单元周围的单元
    {
      // 如果elemNum_tp单元周围存在编号为-1的单元，则直接跳过该单元
      if (elem_tp.get_neig(i) == -1)
        continue;
      // 如果elemNum_tp单元周围的单元未被判断过并且不在当前待判断列表中，并且该单元包含node_num，则将该单元压入elemNum_IncludeNode中
      if ((std::find(elemNum_wait.begin(), elemNum_wait.end(), elem_tp.get_neig(i)) == elemNum_wait.end()) &&
          (std::find(elemNum_succ.begin(), elemNum_succ.end(), elem_tp.get_neig(i)) == elemNum_succ.end()) &&
          (ElemIncludeNode(elem->at(elem_tp.get_neig(i)), nodeNum_wait) != -1))
      {
        elemNum_IncludeNode->push_back(elem_tp.get_neig(i));
        elemNum_wait.push_back(elem_tp.get_neig(i));
      }
    }
    elemNum_succ.push_back(elemNum_tp);
  }
  return;
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

// 查找包含某条边的所有网格单元
void FindShell(std::vector<ELEM> *elem, std::vector<NODE> *node, EDGE edge_tp, std::vector<int> *elemNum_IncludeEdge, std::string judge)
{
  int Edge_F_Node = edge_tp.get_form(0); // 定义一个变量，用来储存edge_tp边的第一个节点
  int Edge_S_Node = edge_tp.get_form(1); // 定义一个变量，用来储存edge_tp边的第二个节点
  std::vector<int> elemNum_IncludeNode;  // 声明一个容器，用来储存包含某节点的所有网格单元
  if (judge == "fast")
    FindBall_fast(elem, node, Edge_F_Node, &elemNum_IncludeNode); // 查找包含edge_tp边第一个节点的所有网格单元
  else if (judge == "slow")
    FindBall_slow(elem, node, Edge_F_Node, &elemNum_IncludeNode); // 查找包含edge_tp边第一个节点的所有网格单元
  // 检索elemNum_IncludeNode容器内所有值，查找包含edge_tp边的第二个节点的网格单元
  for (std::vector<int>::iterator elemNum_iter = elemNum_IncludeNode.begin(); elemNum_iter != elemNum_IncludeNode.end(); ++elemNum_iter)
    if (ElemIncludeNode(elem->at(*elemNum_iter), Edge_S_Node) != -1)
      elemNum_IncludeEdge->push_back(*elemNum_iter);
  return;
}

// 更新包含某一条边的网格单元的相邻关系
void Update_Djacency(std::vector<ELEM> *elem, std::vector<NODE> *node, EDGE edge_tp, std::string judge)
{
  std::vector<int> elemNum_IncludeEdge;                        // 声明一个容器，荣来储存包含edge_tp边的网格单元编号
  FindShell(elem, node, edge_tp, &elemNum_IncludeEdge, judge); // 查找包含edge_tp边的所有网格单元
  if (elemNum_IncludeEdge.size() > 2)                          // 正常情况下问题域内部一条边只会同时被两个网格单元包含，若超过则代表网格生成中出了问题
  {
    std::cout << "Update djacency error: the edge formed by node " << edge_tp.get_form(0) << " and node "
              << edge_tp.get_form(1) << std::endl;
    exit(-1);
  }
  else if (elemNum_IncludeEdge.size() == 2) // 正常边界边
  {
    elem->at(elemNum_IncludeEdge[0]).tr_neig(Face_Opposite_Node(elem->at(elemNum_IncludeEdge[0]), edge_tp), elemNum_IncludeEdge[1]);
    elem->at(elemNum_IncludeEdge[1]).tr_neig(Face_Opposite_Node(elem->at(elemNum_IncludeEdge[1]), edge_tp), elemNum_IncludeEdge[0]);
  }
  else if (elemNum_IncludeEdge.size() == 1) // 初始Delaunay三角化四边形边框边
    elem->at(elemNum_IncludeEdge[0]).tr_neig(Face_Opposite_Node(elem->at(elemNum_IncludeEdge[0]), edge_tp), -1);
  return;
}

// 圆形内边界密度，给定一个节点以及圆形内部边界密度控制信息，判断该节点是否在圆形内部边界内，若存在则返回true，并给节点的spac赋值，若不在则返回false
bool Round_AssignPoint_spac(NODE *node_tp, double *density_control)
{
  double center[2] = {*(density_control + 1), *(density_control + 2)}; // 定义一个变量，储存圆形圆心位置
  double radius = *(density_control + 3);                              // 定义一个变量，储存圆形半径
  double L_Density = *(density_control + 4);                           // 定义一个变量，储存低密度信息
  double H_Density = *(density_control + 5);                           // 定义一个变量，储存高密度信息
  double radius_out = *(density_control + 6);                          // 定义一个变量，储存外部圆形半径
  double distance;                                                     // 声明一个变量，储存点到圆形距离
  if ((distance = getDist(center, node_tp->get_pos())) <= radius)
  {
    node_tp->tr_spac(distance * (H_Density - L_Density) / radius + L_Density);
    return true;
  }
  else if (distance <= radius_out)
  {
    node_tp->tr_spac((distance - radius) * (L_Density - H_Density) / (radius_out - radius) + H_Density);
    return true;
  }
  return false;
}

// 单点密度，给定一个节点以及单点密度控制信息，判断该节点是否在单点外包围圆形内，若存在则返回true，并给节点的spac赋值，若不在则返回false
bool Point_AssignPoint_spac(NODE *node_tp, double *density_control)
{
  double point[2] = {*(density_control + 1), *(density_control + 2)}; // 定义一个变量，储存单点位置
  double L_Density = *(density_control + 3);                          // 定义一个变量，储存低密度信息
  double H_Density = *(density_control + 4);                          // 定义一个变量，储存高密度信息
  double radius_out = *(density_control + 5);                         // 定义一个变量，储存外部圆形半径
  double distance;                                                    // 声明一个变量，储存待判断点到单点距离
  if ((distance = getDist(point, node_tp->get_pos())) <= radius_out)
  {
    node_tp->tr_spac(distance * (L_Density - H_Density) / radius_out + H_Density);
    return true;
  }
  return false;
}

// 内部点生成
void CreateFieldNodes(std::vector<ELEM> *elem, int *elem_num, std::vector<NODE> *node, int *node_num,
                      std::vector<NODE> *node_Insert, double *density_control, std::vector<int> *elemNum_IncludeNodeInitial)
{
  NODE node_tp; // 声明一个临时储存待插入节点的结构体
  int cnt = 0;  // 定义一个变量，来对网格编号进行计数
  bool sign;    // 定义一个变量，用于判断
  // 读取总密度信息，并使General_spac指向正常内部边界图形的周围节点密度控制信息
  double General_spac = *(++density_control);
  density_control = density_control + 1;
  int pointer = 0; // 定义一个变量，用来表示density_control当前指向
  // 声明一个变量，用来储存内部边界图形数
  int figure_cnt = 0;
  // 声明一个变量，对应第(figure_cnt-1)个内部边界图形的种类
  int *figure_type = new int[100];
  do
  {
    *(figure_type + figure_cnt) = int(*(density_control + pointer));
    switch (*(figure_type + figure_cnt))
    {
    case 1: // 正方形边界
      pointer += 9;
      break;
    case 2: // 圆形边界
      pointer += 7;
      break;
    case 3: // 多边形边界
      break;
    case 4: // 直线边界
      break;
    case 5: // 单点
      pointer += 6;
      break;
    default:
      break;
    }
    figure_cnt++;
  } while (*(density_control + pointer) != -1);
  for (std::vector<ELEM>::iterator elem_iter = elem->begin(); elem_iter != elem->end(); ++elem_iter)
  {
    if (elem_iter->get_DeFrame_Grid()) // 如果该单元是初始Delaunay三角化方形边框的四个顶角节点与边界点形成的单元，则跳过
    {
      cnt++;
      continue;
    }
    InitNode(&node_tp); // 初始化node_tp
    node_tp.tr_pos(0, (node->at(elem_iter->get_form(0)).get_pos(0) + node->at(elem_iter->get_form(1)).get_pos(0) +
                       node->at(elem_iter->get_form(2)).get_pos(0)) /
                          3);
    node_tp.tr_pos(1, (node->at(elem_iter->get_form(0)).get_pos(1) + node->at(elem_iter->get_form(1)).get_pos(1) +
                       node->at(elem_iter->get_form(2)).get_pos(1)) /
                          3);
    // 依据输入信息，判断当前node节点位置，并给node的spac赋值
    pointer = 0; // 使用该指针
    sign = false;
    // 判断每个内部边界图形，并得到node节点的spac信息是否成功修改的标识，用sign去接收，若判断完毕sign仍然是false，则直接给当前节点node的spac赋值General_spac
    // 给每个函数传递参数时，直接传递指向第i个内部边界图形种类位置的指针
    for (int i = 0; i < figure_cnt; i++)
    {
      switch (*(figure_type + i))
      {
      case 1: // 正方形边界
        pointer += 9;
        break;
      case 2: // 圆形边界
        sign = Round_AssignPoint_spac(&node_tp, density_control + pointer);
        pointer += 7;
        break;
      case 3: // 多边形边界
        break;
      case 4: // 直线边界
        break;
      case 5: // 单点
        sign = Point_AssignPoint_spac(&node_tp, density_control + pointer);
        pointer += 6;
        break;
      default:
        break;
      }
      if (sign)
        break;
    }
    if (!sign)
      node_tp.tr_spac(General_spac);
    if ((getDist(node_tp.get_pos(), node->at(elem_iter->get_form(0)).get_pos()) >= node_tp.get_spac()) &&
        (getDist(node_tp.get_pos(), node->at(elem_iter->get_form(1)).get_pos()) >= node_tp.get_spac()) &&
        (getDist(node_tp.get_pos(), node->at(elem_iter->get_form(2)).get_pos()) >= node_tp.get_spac()))
    {
      node_Insert->push_back(node_tp);              // 将生成的节点插入node_Insert
      elemNum_IncludeNodeInitial->push_back(cnt++); // 将包含该生成节点的网格单元编号插入elemNum_IncludeNodeInitial
    }
  }
  delete[] figure_type;
  return;
}

// 内部点插入
bool InsertFieldPoint(std::vector<ELEM> *elem, int *elem_num, std::vector<NODE> *node, int *node_num,
                      NODE node_Insert, int elemNum_IncludeNodeIns)
{
  ELEM elem_tp;                                  // 声明一个用于临时储存生成网格单元的变量
  EDGE edge_tp;                                  // 创建一个结构体，用于临时储存生成的边
  std::vector<int> elemNum_Cav;                  // 创建一个空腔容器，用来储存外接圆包含待插入节点node_Insert的网格单元编号
  elemNum_Cav.push_back(elemNum_IncludeNodeIns); // 压入初始的外接圆包含待插入节点的网格单元编号
  // 空腔查找
  FindElemCav(elem, elem_num, node, node_num, &elemNum_Cav, node_Insert, elemNum_IncludeNodeIns);
  // RepairCavity(elem, elem_num, node, node_num, &elemNum_Cav, node_Insert);//空腔修复
  std::vector<EDGE> edge_Cav; // 创建一个容器，用来储存空腔边界，一个边由两个节点构成
  /**
   * * 在edge_Cav内查找当前搜索的网格单元边界，如果搜到end()，代表没有找到，该边是空腔边，可以插入；
   * * 如果没有搜到搜到end()，代表该边不是空腔边，不予插入，并且在edge_Cav内删除该边
   */
  // 将所有空腔边界压入edge_Cav
  for (unsigned long long i = 0; i < elemNum_Cav.size(); i++)
  {
    InitEdge(&edge_tp);                 // 初始化edge_tp
    int elemNum_tp = elemNum_Cav.at(i); // 取出空腔内一个网格单元
    // 定义一个变量，储存网格单元所有边，便于后续对边的访问
    int edge_Cavtp[3][2] = {{elem->at(elemNum_tp).get_form(0), elem->at(elemNum_tp).get_form(1)},  // 显然一个网格单元有三条边
                            {elem->at(elemNum_tp).get_form(1), elem->at(elemNum_tp).get_form(2)},  // 取节点编号的时候从小到大排列
                            {elem->at(elemNum_tp).get_form(0), elem->at(elemNum_tp).get_form(2)}}; // 便于后边各种查找
    // 一个网格单元有三条边
    for (int j = 0; j < 3; j++)
    {
      InitEdge(&edge_tp); // 初始化edge_tp
      edge_tp.tr_form(0, edge_Cavtp[j][0]);
      edge_tp.tr_form(1, edge_Cavtp[j][1]);
      std::vector<EDGE>::iterator edge_Iter; // 创建一个迭代器，便于删除非空腔边界边
      if ((edge_Iter = std::find(edge_Cav.begin(), edge_Cav.end(), edge_tp)) == edge_Cav.end())
      {
        // 首先根据网格单元边理想长度判断该node_Insert是否能插入
        if ((getDist(node_Insert.get_pos(), node->at(edge_tp.get_form(0)).get_pos()) >= node_Insert.get_spac()) &&
            (getDist(node_Insert.get_pos(), node->at(edge_tp.get_form(1)).get_pos()) >= node_Insert.get_spac()))
          // 显然，该边没有被插入过elemNum_edge，说明该边是空腔边界边，并且符合密度控制条件，插入边
          edge_Cav.push_back(edge_tp);
        else // 不符合网格单元边理想长度，不予插入该节点，返回false
          return false;
      }
      else
        edge_Cav.erase(edge_Iter); // 显然，该边被插入过elemNum_edge，说明该边不是空腔边界边，删除边
    }
  }
  InitElemCav(elem, &elemNum_Cav); // 空腔初始化操作，在elem内中初始化空腔所包含的网格单元
  // 对每个空腔边界进行操作，生成待插入点与当前空腔边界形成的网格单元，首先更新空腔边界边的相邻关系
  for (unsigned long long i = 0; i < edge_Cav.size(); i++)
  {
    // 前elemNum_Cav.size()个边与待插入点生成的网格单元编号采用空腔中删除的网格单元编号
    if (i < elemNum_Cav.size())
    {
      int elemNum_tp = elemNum_Cav.at(i); // 取出空腔内一个网格单元编号
      // 显然可以直接给新生成的网格单元赋节点编号
      elem->at(elemNum_tp).tr_form(0, edge_Cav.at(i).get_form(0));
      elem->at(elemNum_tp).tr_form(1, edge_Cav.at(i).get_form(1));
      elem->at(elemNum_tp).tr_form(2, *node_num); // 最大的节点编号永远是待插入的节点编号
      // 改变当前网格中包含该节点的任意单元标号
      node->at(edge_Cav.at(i).get_form(0)).tr_elem(elemNum_tp);
      node->at(edge_Cav.at(i).get_form(1)).tr_elem(elemNum_tp);
      // node->at(*node_num).tr_elem(elemNum_tp);//由于是先检查能不能插入该节点，再将该节点插入node容器，所以此处一定会越界
      Update_Djacency(elem, node, edge_Cav.at(i), "slow"); // 查找包含该边的所有网格单元，并更新相邻关系
    }
    // 之后生成的网格全压入elem中，并赋予新的编号
    else
    {
      InitElem(&elem_tp); // 初始化elem_tp
      // 显然可以直接给新生成的网格单元赋节点编号
      elem_tp.tr_form(0, edge_Cav.at(i).get_form(0));
      elem_tp.tr_form(1, edge_Cav.at(i).get_form(1));
      elem_tp.tr_form(2, *node_num); // 最大的节点编号永远是待插入的节点编号
      // 改变当前网格中包含该节点的任意单元标号
      node->at(edge_Cav.at(i).get_form(0)).tr_elem(*elem_num);
      node->at(edge_Cav.at(i).get_form(1)).tr_elem(*elem_num);
      // node->at(*node_num).tr_elem(*elem_num); // 由于是先检查能不能插入该节点，再将该节点插入node容器，所以此处一定会越界
      elem->push_back(elem_tp); // 将elem_tp1压入elem中，形成网格单元的生成
      *elem_num += 1;
      Update_Djacency(elem, node, edge_Cav.at(i), "slow"); // 查找包含该边的所有网格单元，并更新相邻关系
    }
  }
  InitEdge(&edge_tp);                 // 初始化elem_tp
  edge_tp.tr_form(1, *node_num);      // 显然待更新边的一个节点总是待插入节点，并且其编号也总是最大
  std::vector<int> elemNum_edgeJudge; // 创建一个容器，用来储存检查过的空腔边节点
  // 更新插入点与空腔边节点连接形成的边的相邻关系
  for (unsigned long long i = 0; i < edge_Cav.size(); i++)
  {
    // 注意，一个边有2个节点
    // 节点一
    if (find(elemNum_edgeJudge.begin(), elemNum_edgeJudge.end(), edge_Cav.at(i).get_form(0)) == elemNum_edgeJudge.end())
    {
      // 显然，该节点没有被检查过，插入节点，更新该节点形成边的相邻关系
      edge_tp.tr_form(0, edge_Cav.at(i).get_form(0));
      Update_Djacency(elem, node, edge_tp, "slow");
      elemNum_edgeJudge.push_back(edge_Cav.at(i).get_form(0)); // 插入该节点编号到elemNum_edgeJudge
      if (elemNum_edgeJudge.size() == edge_Cav.size())         // 显然只会形成与空腔边界边数量相同的边数量，达到该数量则直接退出循环
        break;
    }
    // 节点二
    if (find(elemNum_edgeJudge.begin(), elemNum_edgeJudge.end(), edge_Cav.at(i).get_form(1)) == elemNum_edgeJudge.end())
    {
      // 显然，该节点没有被检查过，插入节点，更新该节点形成边的相邻关系
      edge_tp.tr_form(0, edge_Cav.at(i).get_form(1));
      Update_Djacency(elem, node, edge_tp, "slow");
      elemNum_edgeJudge.push_back(edge_Cav.at(i).get_form(1)); // 插入该节点编号到elemNum_edgeJudge
      if (elemNum_edgeJudge.size() == edge_Cav.size())         // 显然只会形成与空腔边界边数量相同的边数量，达到该数量则直接退出循环
        break;
    }
  }
  return true;
}

// 给定两条边，判断该两条边是否相交。此处使用Shewchuk的orient2d几何谓词：若两条边相交，则第一条边的两个节点在第二条边的位置关系不同，计算结果的符号相反，反之亦然
bool Boundary_Intersection(std::vector<NODE> *node, EDGE edge_1, EDGE edge_2)
{
  // 取出两条边的节点坐标，赋值给两个数组
  double edge_1_pos[][2] = {{node->at(edge_1.get_form(0)).get_pos(0), node->at(edge_1.get_form(0)).get_pos(1)},
                            {node->at(edge_1.get_form(1)).get_pos(0), node->at(edge_1.get_form(1)).get_pos(1)}};
  double edge_2_pos[][2] = {{node->at(edge_2.get_form(0)).get_pos(0), node->at(edge_2.get_form(0)).get_pos(1)},
                            {node->at(edge_2.get_form(1)).get_pos(0), node->at(edge_2.get_form(1)).get_pos(1)}};
  // 如果该值>0代表edge_2的两个节点在线段edge_1的位置关系相同，即不相交
  // * 此处认为，如果一条边有一个节点在另一条边上时，也判定为不相交
  if (orient2d(edge_1_pos[0], edge_1_pos[1], edge_2_pos[0]) * orient2d(edge_1_pos[0], edge_1_pos[1], edge_2_pos[1]) >= 0)
    return false;
  else
  {
    // 还要判断edge_1的两个节点在线段edge_2的位置关系，以确保结果准确
    if (orient2d(edge_2_pos[0], edge_2_pos[1], edge_1_pos[0]) * orient2d(edge_2_pos[0], edge_2_pos[1], edge_1_pos[1]) >= 0)
      return false;
    else
      return true;
  }
  return false;
}

// 给定两个必定相邻的网格单元编号，判断该两个网格单元形成的四边形是否是凹四边形
bool Concave_Quadrilateral(std::vector<ELEM> *elem, int elemNum_tp_1, int elemNum_tp_2, std::vector<NODE> *node)
{
  EDGE edge_tp_1, edge_tp_2; // 声明两个变量，第一个储存该两个网格单元相邻边，第二个储存该两个网格单元相对节点形成的边
  edge_tp_1 = elem_AdjacentEdge(elem->at(elemNum_tp_1), elemNum_tp_2);
  // 现在取得该两个网格单元相对节点形成的边
  edge_tp_2 = elem_OppositeEdge(elem, elemNum_tp_1, elemNum_tp_2);
  // edge_tp_2.tr_form(0, elem->at(elemNum_tp_1).get_form(Face_Opposite_Node(elem->at(elemNum_tp_1), edge_tp_1)));
  // edge_tp_2.tr_form(1, elem->at(elemNum_tp_2).get_form(Face_Opposite_Node(elem->at(elemNum_tp_2), edge_tp_1)));
  // edge_tp_2.Swap(); // 使两个节点编号从小到大排序
  //  如果该edge_tp_1, edge_tp_2相交，则代表该四边形不是凹四边形
  if (Boundary_Intersection(node, edge_tp_1, edge_tp_2))
    return false;
  else
    return true;
}

// 给点一个边界，在当前三角化内查找从节点node_s到节点 node_e的路径
void FindPath(std::vector<ELEM> *elem, std::vector<NODE> *node, EDGE border_edge, std::vector<int> *path_set)
{
  // 定义两个变量，分别储存待恢复边界边的起始节点和终止节点编号
  int node_s = border_edge.get_form(0), node_e = border_edge.get_form(1);
  std::vector<int> elemNum_IncludeNode_s;                    // 声明一个容器，用来储存包含起始节点node_s的网格单元
  FindBall_fast(elem, node, node_s, &elemNum_IncludeNode_s); // 查找所有包含起始节点的网格单元
  // 遍历elemNum_IncludeNode_s容器，判断每个网格单元中与起始节点node_s相对的边是否与待恢复边border_edge相交
  // 用elemNum_Intersect储存相交单元的编号，并且在elemNum_IncludeNode_s中有且只有一个网格单元会与待恢复边border_edge相交
  int elemNum_Intersect = -1; // 用-1初始化elemNum_Intersect，代表当前未找到相交单元
  for (std::vector<int>::iterator iter = elemNum_IncludeNode_s.begin(); iter != elemNum_IncludeNode_s.end();)
  {
    // 判断当前边是否与恢复边border_edge相交
    if (Boundary_Intersection(node, Node_Opposite_Face(elem->at(*iter), node_s), border_edge))
      elemNum_Intersect = *iter;
    // 找到相交单元了
    if (elemNum_Intersect != -1)
      break;
    iter++;
  }
  // 如果该值还是-1，则代表第一个路径元查找失败，退出程序并报错
  if (elemNum_Intersect == -1)
  {

    std::cout << "The first pathl search failed, please check the program!" << std::endl;
    exit(-1);
  }
  // 正常情况下，将找到的单元编号压入path_set容器
  path_set->push_back(elemNum_Intersect);
  int elemNum_old;                                                         // 声明一个变量，用来储存上次判断的网格单元编号
  int elemNum_tp;                                                          // 声明一个变量，用来储存当前判断的网格单元编号
  ELEM elem_tp;                                                            // 声明一个变量，用来储存当前判断的网格单元
  EDGE edge_old = Node_Opposite_Face(elem->at(elemNum_Intersect), node_s); // 定义一个变量，用来储存上次判断的边变量
  EDGE edge_tp_1, edge_tp_2, edge_tp_3;                                    // 声明三个变量，用来储存当前判断的网格单元的三条边
  // 依据第一个路径元，查找剩下路径元
  do
  {
    // 取出容器内最后一个数据所指向的网格单元
    elemNum_old = path_set->back();
    // 利用该网格单元（即上次判断的网格单元）中 上次判断与待恢复边border_edge相交的边
    //  来得到 与该网格单元 以该边作为相邻边 的网格单元编号，也就是下一次压入path_set容器的单元编号
    elemNum_tp = elem->at(elemNum_old).get_neig(Face_Opposite_Node(elem->at(elemNum_old), edge_old));
    InitElem(&elem_tp);
    elem_tp = elem->at(elemNum_tp);
    // 生成当前判断的网格单元的三条边
    edge_tp_1.tr_form(0, elem_tp.get_form(0));
    edge_tp_1.tr_form(1, elem_tp.get_form(1));
    // 第二条边
    edge_tp_2.tr_form(0, elem_tp.get_form(0));
    edge_tp_2.tr_form(1, elem_tp.get_form(2));
    // 第三条边
    edge_tp_3.tr_form(0, elem_tp.get_form(1));
    edge_tp_3.tr_form(1, elem_tp.get_form(2));
    // 判断每一条边，首先排除上次判断的边edge_old
    if ((edge_tp_1 != edge_old) && Boundary_Intersection(node, edge_tp_1, border_edge))
      edge_old = edge_tp_1;
    else if ((edge_tp_2 != edge_old) && Boundary_Intersection(node, edge_tp_2, border_edge))
      edge_old = edge_tp_2;
    else if ((edge_tp_3 != edge_old) && Boundary_Intersection(node, edge_tp_3, border_edge))
      edge_old = edge_tp_3;
    // 将当前判断单元编号压入path_set容器，作为路径元
    path_set->push_back(elemNum_tp);
    // 最后判断当前网格单元是否包含待恢复边界边的终止节点编号node_e，若包含，则代表路径查找完毕，退出循环
  } while (ElemIncludeNode(elem_tp, node_e) == -1);
  return;
}

// 给定两个网格单元编号，实现两个网格单元的对角交换
void TwoElem_DiagonalSwap(std::vector<ELEM> *elem, std::vector<NODE> *node, int elemNum_F, int elemNum_S)
{
  // 定义两个变量，储存待连接的两个节点编号
  int nodeNum_F_Link = elem->at(elemNum_F).get_form(AdjacentElem_pos(elem->at(elemNum_F), elemNum_S));
  int nodeNum_S_Link = elem->at(elemNum_S).get_form(AdjacentElem_pos(elem->at(elemNum_S), elemNum_F));
  // 声明两个数组变量，储存待删除连接关系的两个节点在指定网格单元的form中位置
  int nodeNum_inF_Form[2]; // 编号为elemNum_F
  int nodeNum_inS_Form[2]; // 编号为elemNum_S
  if (AdjacentElem_pos(elem->at(elemNum_F), elemNum_S) == 0)
    nodeNum_inF_Form[0] = 1, nodeNum_inF_Form[1] = 2;
  else if (AdjacentElem_pos(elem->at(elemNum_F), elemNum_S) == 1)
    nodeNum_inF_Form[0] = 0, nodeNum_inF_Form[1] = 2;
  else
    nodeNum_inF_Form[0] = 0, nodeNum_inF_Form[1] = 1;
  if (AdjacentElem_pos(elem->at(elemNum_S), elemNum_F) == 0)
    nodeNum_inS_Form[0] = 1, nodeNum_inS_Form[1] = 2;
  else if (AdjacentElem_pos(elem->at(elemNum_S), elemNum_F) == 1)
    nodeNum_inS_Form[0] = 0, nodeNum_inS_Form[1] = 2;
  else
    nodeNum_inS_Form[0] = 0, nodeNum_inS_Form[1] = 1;
  // 声明两个变量，储存待删除连接关系的两个节点编号，且**Large>**Small，并初始化为-1，即不指向任何节点
  int nodeNum_Delete_S = elem->at(elemNum_F).get_form(nodeNum_inF_Form[0]),
      nodeNum_Delete_L = elem->at(elemNum_F).get_form(nodeNum_inF_Form[1]);
  // 声明一个变量，储存相邻单元编号以及相邻单元的form位置
  // 并且很显然两个网格单元只会有4条与其他网格单元相邻的边
  int Num_TP[4][2];
  // 边1
  Num_TP[0][0] = elem->at(elemNum_F).get_neig(nodeNum_inF_Form[0]);
  if (Num_TP[0][0] != -1)
    Num_TP[0][1] = AdjacentElem_pos(elem->at(elem->at(elemNum_F).get_neig(nodeNum_inF_Form[0])), elemNum_F);
  // 边3
  Num_TP[1][0] = elem->at(elemNum_S).get_neig(nodeNum_inS_Form[0]);
  if (Num_TP[1][0] != -1)
    Num_TP[1][1] = AdjacentElem_pos(elem->at(elem->at(elemNum_S).get_neig(nodeNum_inS_Form[0])), elemNum_S);
  // 边3
  Num_TP[2][0] = elem->at(elemNum_F).get_neig(nodeNum_inF_Form[1]);
  if (Num_TP[2][0] != -1)
    Num_TP[2][1] = AdjacentElem_pos(elem->at(elem->at(elemNum_F).get_neig(nodeNum_inF_Form[1])), elemNum_F);
  // 边4
  Num_TP[3][0] = elem->at(elemNum_S).get_neig(nodeNum_inS_Form[1]);
  if (Num_TP[3][0] != -1)
    Num_TP[3][1] = AdjacentElem_pos(elem->at(elem->at(elemNum_S).get_neig(nodeNum_inS_Form[1])), elemNum_S);
  // 首先在容器elem内修改两个网格单元form信息，实现对角交换
  // *此处没有考虑网格单元的DeFrame_Grid值，不过一般情况下不用考虑
  elem->at(elemNum_F).tr_form(nodeNum_F_Link, nodeNum_S_Link, nodeNum_Delete_L);
  elem->at(elemNum_F).tr_neig(Num_TP[1][0], Num_TP[0][0], elemNum_S);
  elem->at(elemNum_F).Sort();
  Renew_NodeElem(elem, node, elemNum_F);
  if (Num_TP[0][0] != -1)
    elem->at(Num_TP[0][0]).tr_neig(Num_TP[0][1], elemNum_F);
  if (Num_TP[1][0] != -1)
    elem->at(Num_TP[1][0]).tr_neig(Num_TP[1][1], elemNum_F);
  elem->at(elemNum_S).tr_form(nodeNum_F_Link, nodeNum_S_Link, nodeNum_Delete_S);
  elem->at(elemNum_S).tr_neig(Num_TP[3][0], Num_TP[2][0], elemNum_F);
  elem->at(elemNum_S).Sort();
  Renew_NodeElem(elem, node, elemNum_S);
  if (Num_TP[2][0] != -1)
    elem->at(Num_TP[2][0]).tr_neig(Num_TP[2][1], elemNum_S);
  if (Num_TP[3][0] != -1)
    elem->at(Num_TP[3][0]).tr_neig(Num_TP[3][1], elemNum_S);
  return;
}

// 给定一个边界，通过不断对角交换实现边界恢复
void Swap_BoundaryRecovery(std::vector<ELEM> *elem, std::vector<NODE> *node, EDGE border_edge)
{
  // 定义两个变量，分别储存待恢复边界边的起始节点和终止节点编号
  int node_s = border_edge.get_form(0), node_e = border_edge.get_form(1);
  // 首先要确定与待恢复边界相交的三角形单元，它们的集合称为路径（path），它们自身称为路径元（pathl），
  std::vector<int> path_set;                    // 声明一个容器，按从节点node_s到节点 node_e的顺序储存所有路径元
  FindPath(elem, node, border_edge, &path_set); // 查找路径
  /*
   * 路径查找完毕后，开始按路径顺序一个个恢复边界，每次提出路径容器path_set的前两个网格单元编号，并按以下规则实行边界恢复：
   * 1.首先判断该两个网格单元组成的四边形是否是凹四边形，由于凹四边形的对角线必定不相交，所以只需要判断该四边形对角线是否相交，若相交则禁止此次对角交换，对下一个四边形进行判断。
   * 2.若执行对角交换后，新形成的对角线仍然与待恢复边相交，则延后此次对角交换，对下一个四边形进行判断。
   * 3.若当前四边形不被规则1、2阻止，则正常进行对角交换，接着对下一个四边形进行判断。
   * 4.若当前路径内所有对角交换都被规则1、2所阻止，则先对被规则2延后的四边形进行对角交换，然后接着对对下一个四边形进行判断。
   * 5.重复以上操作，直到路径path_set为空，即恢复了边界边。
   * path_set容器内的数据顺序不得随意改变！
   */
  std::vector<int>::iterator iter; // 声明一个容器path_set的迭代器
  // 当path_set容器内有两个以上元素时，如果当前循环删除的是最后两个元素，删除后iter不会指向path_set内的值，而是指向被删除的值，此时再判断相邻关系会产生错误
  // 定义一个变量，用来解决上述问题。初始化为0，代表当前循环删除的不是是最后两个元素
  int iter_adapt = 0;
  bool whether_Swap_current = true; // 定义一个变量，用来判断当前遍历path_set容器时是否成功执行过对角交换初始化为 是
  bool whether_Swap_last = true;    // 定义一个变量，用来判断上次遍历path_set容器时是否成功执行过对角交换初始化为 是
  // 定义一个迭代器，指向每轮for循环中第一个被规则二所延后的网格单元编号位置，初始化为path_set.begin()
  std::vector<int>::iterator iter_rule_two = path_set.begin();
  // 定义一个变量，判断当前循环中iter_rule_two有没有被赋过值，保证iter_rule_two指向每轮for循环中第一个被规则二所延后的网格单元编号位置，初始化为false
  bool iter_rule_two_judge = false;
  // 定义一个变量，判断当前循环操作的两个网格单元是否是规则4所描述的网格单元，初始化为false
  bool rule_four_judge = false;
  int elemNum_tp_1, elemNum_tp_2; // 声明两个变量，储存当前判断的两个网格单元编号
  // 只要路径path_set不为空，则继续执行边界恢复
  while (!path_set.empty())
  {
    //  如果path_set内只有一个元素，说明边界恢复错误，退出程序
    if (path_set.size() == 1)
    {
      std::cout << "Boundary recovery error, exit program!" << std::endl;
      exit(-1);
    }
    // 遍历path_set的所有元素（由于当前处理特性，每次取出两个元素判断，则迭代器结尾应该设为path_set.end()-1）
    // 遍历一遍后，并且上次遍历没有执行过任何对角交换操作的情况下，剩下的所有网格单元都被规则1、2所阻止，所以下次遍历先从被规则2阻止的网格单元开始
    iter = whether_Swap_last == true ? path_set.begin() : iter_rule_two;
    whether_Swap_current = false; // 重新开始判断
    iter_rule_two_judge = false;  // 重新开始判断
    for (; iter != path_set.end() - 1;)
    {
      // 取出容器path_set的前两个网格单元编号
      elemNum_tp_1 = *iter;
      elemNum_tp_2 = *(iter + 1);
      // 如果上次没有执行过对角交换并且当前for循环没有执行过对角交换，则尽管当前iter指向的网格单元与下一个网格单元形成的四边形是凹四边形，依然执行对角交换
      if (!whether_Swap_last && !whether_Swap_current)
        // 显然当前操作的两个网格单元是规则4所描述的网格单元，这两个网格单元完成对角交换后依然同时与待恢复边界边border_edge相交，所以需要同时插入两个网格单元编号
        rule_four_judge = true;
      else
      {
        // 首先判断该两个网格单元组成的四边形是否是凹四边形
        if (Concave_Quadrilateral(elem, elemNum_tp_1, elemNum_tp_2, node))
        {
          ++iter;
          continue;
        }
        if (Boundary_Intersection(node, elem_OppositeEdge(elem, elemNum_tp_1, elemNum_tp_2), border_edge))
        {
          if (!iter_rule_two_judge)
          {
            iter_rule_two = iter;
            iter_rule_two_judge = true;
          }
          ++iter;
          continue;
        }
      }
      // 若当前循环能运行到这里，代表当前两个网格单元能执行对角交换
      TwoElem_DiagonalSwap(elem, node, elemNum_tp_1, elemNum_tp_2);
      whether_Swap_current = true; // 当前for循环中成功执行过对角交换
      // 从path_set中删除这两个元素
      iter = path_set.erase(iter);
      // 判断当前循环删除的是否是最后两个元素
      if (iter == path_set.end() - 1)
        iter_adapt = 1;
      else
        iter_adapt = 0;
      iter = path_set.erase(iter);
      // 如果路径容器path_set为空，代表边界恢复结束，直接退出循环
      if (path_set.empty())
        break;
      // 由于要保证路径的完整性，需要从执行对角交换的两个网格单元中找出与当前iter指向的网格单元相邻的网格单元，并插入到iter指向的网格单元之前一个位置
      if (AdjacentElem_pos(elem->at(elemNum_tp_1), *(iter - iter_adapt)) != -1)
      {
        path_set.insert(iter, elemNum_tp_1);
        if (rule_four_judge)
        {
          path_set.insert(iter, elemNum_tp_2);
          // 由于iter会指向刚加入的元素，所以需要加一位来防止对两个单元重复判断
          ++iter;
        }
      }
      else
      {
        if (rule_four_judge)
          path_set.insert(iter, elemNum_tp_1);
        path_set.insert(iter, elemNum_tp_2);
        if (rule_four_judge)
          ++iter;
      }
      rule_four_judge = false; // 重置判断
    }
    whether_Swap_last = whether_Swap_current;
  }
  return;
}

// 在node容器内交换两个节点位置，并更新网格相邻信息
void ReplaceNode_two(std::vector<ELEM> *elem, std::vector<NODE> *node, int nodeNum_one, int nodeNum_two)
{
  std::vector<int> elemNum_NodeOne;                         // 创建一个容器，用来储存包含第一个节点的所有网格单元编号
  std::vector<int> elemNum_NodeTwo;                         // 创建一个容器，用来储存包含第二个节点的所有网格单元编号
  FindBall_fast(elem, node, nodeNum_one, &elemNum_NodeOne); // 查找包含第一个节点的所有网格单元编号
  FindBall_fast(elem, node, nodeNum_two, &elemNum_NodeTwo); // 查找包含第二个节点的所有网格单元编号
  // 修改elem容器内值
  for (std::vector<int>::iterator iter = elemNum_NodeOne.begin(); iter != elemNum_NodeOne.end(); ++iter)
  {
    elem->at(*iter).tr_form(ElemIncludeNode(elem->at(*iter), nodeNum_one), nodeNum_two);
  }
  for (std::vector<int>::iterator iter = elemNum_NodeTwo.begin(); iter != elemNum_NodeTwo.end(); ++iter)
  {
    elem->at(*iter).tr_form(ElemIncludeNode(elem->at(*iter), nodeNum_two), nodeNum_one);
  }
  // 替换节点位置
  NODE node_tp = node->at(nodeNum_one);
  node->at(nodeNum_one) = node->at(nodeNum_two);
  node->at(nodeNum_two) = node_tp;
  return;
}

// 在elem容器内交换两个单元位置，并更新网格相邻信息
void ReplaceElem_two(std::vector<ELEM> *elem, std::vector<NODE> *node, int elemNum_one, int elemNum_two)
{
  std::vector<int> elemNum_ElemOne; // 创建一个容器，用来储存第一个网格单元的相邻网格单元编号
  std::vector<int> elemNum_ElemTwo; // 创建一个容器，用来储存第二个网格单元的相邻网格单元编号
  for (int i = 0; i < DIM + 1; i++)
    if (elem->at(elemNum_one).get_neig(i) != -1)
      elemNum_ElemOne.push_back(elem->at(elemNum_one).get_neig(i));
  for (int i = 0; i < DIM + 1; i++)
    if (elem->at(elemNum_two).get_neig(i) != -1)
      elemNum_ElemTwo.push_back(elem->at(elemNum_two).get_neig(i));
  // 修改elem容器内值
  for (std::vector<int>::iterator iter = elemNum_ElemOne.begin(); iter != elemNum_ElemOne.end(); ++iter)
    elem->at(*iter).tr_neig(AdjacentElem_pos(elem->at(*iter), elemNum_one), elemNum_two);
  for (std::vector<int>::iterator iter = elemNum_ElemTwo.begin(); iter != elemNum_ElemTwo.end(); ++iter)
    elem->at(*iter).tr_neig(AdjacentElem_pos(elem->at(*iter), elemNum_two), elemNum_one);
  // 修改node容器内值
  for (int i = 0; i < DIM + 1; i++)
  {
    node->at(elem->at(elemNum_one).get_form(i)).tr_elem(elemNum_two);
    node->at(elem->at(elemNum_two).get_form(i)).tr_elem(elemNum_one);
  }
  // 替换单元位置
  ELEM elem_tp = elem->at(elemNum_one);
  elem->at(elemNum_one) = elem->at(elemNum_two);
  elem->at(elemNum_two) = elem_tp;
  return;
}

// 在node容器内删除最后一个节点元素
void Removal_LastNode(std::vector<ELEM> *elem, std::vector<NODE> *node, int *node_num)
{
  /*
  std::vector<int> elemNum_IncludeNode;                            // 创建一个容器，用来储存包含待删除节点的所有网格单元编号
  FindBall_fast(elem, node, nodeNum_Remove, &elemNum_IncludeNode); // 查找包含待删除节点的所有网格单元编号
  ELEM elem_tp;                                                    // 储存当前操作单元
  int elemAdNum_tp;                                                // 储存当前操作单元相邻单元编号
  for (std::vector<int>::iterator elemNum_iter = elemNum_IncludeNode.begin(); elemNum_iter != elemNum_IncludeNode.end(); ++elemNum_iter)
  {
    elem_tp = elem->at(*elemNum_iter);                                         // 当前操作网格单元
    elemAdNum_tp = elem_tp.get_neig(ElemIncludeNode(elem_tp, nodeNum_Remove)); // 当前操作单元中nodeNum_Remove节点相对的网格单元编号
    if (elemAdNum_tp == -1)                                                    //-1代表无相邻单元
    {
    }
    else
    {
    elem->at(elemAdNum_tp).tr_neig(ElemAdjacentElem(elem->at(elemAdNum_tp), *elemNum_iter), -1); // 更新相邻信息
    Renew_NodeElem(elem, node, elemAdNum_tp);                                                    // 更新elemAdNum_tp网格单元节点信息
    }
    // 初始化elem_tp，用于初始化*elemNum_iter所指网格单元
    InitElem(&elem_tp);
    elem->at(*elemNum_iter) = elem_tp;
    elem->at(*elemNum_iter).tr_judge(false); // 打个标记，代表当前单元无效
  }
  */
  // 待删除节点永远是node容器内最后一个元素，直接删除就行
  node->pop_back();
  *node_num -= 1;
  return;
}

// 在elem容器内删除最后一个单元并更新所有信息
void Removal_LastElem(std::vector<ELEM> *elem, int *elem_num, std::vector<NODE> *node)
{
  ELEM elem_tp = elem->at(elem->size() - 1); // 定义一个临时变量，储存elem容器内最后一个网格单元元素
  int elemNum_neig;                          // 声明一个变量，用来储存当前网格单元的相邻节点编号
  for (int i = 0; i < DIM + 1; i++)
  {
    elemNum_neig = elem_tp.get_neig(i);
    if (elemNum_neig != -1 && elemNum_neig < elem->size())
    {
      // 先更新待删除单元的所有节点的elem信息
      for (int j = 0; j < DIM + 1; j++)
      {
        if (j == i)
          continue;
        node->at(elem_tp.get_form(j)).tr_elem(elemNum_neig);
      }
      elem->at(elemNum_neig).tr_neig(AdjacentElem_pos(elem->at(elemNum_neig), elem->size() - 1), -1);
    }
  }
  elem->pop_back();
  *elem_num -= 1;
  return;
}

// 在node容器内删除节点与单元
void Removal_NodeElem(std::vector<ELEM> *elem, int *elem_num, std::vector<NODE> *node, int *node_num,
                      std::vector<int> *elemNum_Remove, int *nodeNum_Remove)
{
  // 为简化删除节点流程，当查找到待删除节点时，使用容器内最后一个有效节点来替换此位置，在程序结束时，删除最后cnt_node个节点，最大限度地保证了整个网格的相邻信息不变
  // 为避免不必要的麻烦，从nodeNum_Remove的最后一个节点编号开始检索替换
  int cnt_node = 0;            // 记录待删除节点的个数
  int Last_NodeNum_Remove = 3; // 定义一个变量，储存nodeNum_Remove的最后一个元素位置
  // 将nodeNum_Remove内所有节点放到node容器最后
  for (int i = Last_NodeNum_Remove; i >= 0; i--)
  {
    // 将该节点与node容器的最后一个有效节点交换位置
    // 如果该节点就位于node容器的最后一个有效节点位置，则不交换
    if (*(nodeNum_Remove + i) == node->size() - 1 - cnt_node)
    {
      cnt_node++;
      continue;
    }
    ReplaceNode_two(elem, node, *(nodeNum_Remove + i), node->size() - 1 - cnt_node++);
  }
  int cnt_elem = 0; // 记录待删除网格单元的个数
  // 将elemNum_Remove内所有网格单元放到elem容器最后
  for (int i = elemNum_Remove->size() - 1; i >= 0; i--)
  {
    // 将该网格单元与elem容器的最后一个有效网格单元交换位置
    // 如果该网格单元就位于elem容器的最后一个有效网格单元位置，则不交换
    if (elemNum_Remove->at(i) == elem->size() - 1 - cnt_elem)
    {
      cnt_elem++;
      continue;
    }
    ReplaceElem_two(elem, node, elemNum_Remove->at(i), elem->size() - 1 - cnt_elem++);
  }
  // 由于节点是直接删除，不需要更新相邻信息，所以先删除网格单元
  for (int i = 0; i < cnt_elem; i++)
    Removal_LastElem(elem, elem_num, node);
  for (int i = 0; i < cnt_node; i++)
    Removal_LastNode(elem, node, node_num);
  return;
}

// 支持1x2矩阵与2x1矩阵的乘积运算，返回乘积结果，此处是一个数
double matrix_product_1x2_2x1(double matrix_1[2], double matrix_2[2][1])
{
  return matrix_1[0] * matrix_2[0][0] + matrix_1[1] * matrix_2[1][0];
}

// 单元度量准则，给定三角形三个顶点位置，返回三角形单元形状质量，当且仅当三角形为等边三角形时，quality值为1，其他任意三角形的quality值小于1
double Shape_Quality(NODE node_tp1, NODE node_tp2, NODE node_tp3)
{
  double area = Triangle_area(node_tp1, node_tp2, node_tp3);                                                                // 这里可以无视顶点是否逆时针排序
  double l1_square = pow(node_tp1.get_pos(0) - node_tp2.get_pos(0), 2) + pow(node_tp1.get_pos(1) - node_tp2.get_pos(1), 2); // 第一条边的长度的平方
  double l2_square = pow(node_tp2.get_pos(0) - node_tp3.get_pos(0), 2) + pow(node_tp2.get_pos(1) - node_tp3.get_pos(1), 2); // 第二条边的长度的平方
  double l3_square = pow(node_tp3.get_pos(0) - node_tp1.get_pos(0), 2) + pow(node_tp3.get_pos(1) - node_tp1.get_pos(1), 2); // 第三条边的长度的平方
  double quality = 4 * sqrt(3) * area / (l1_square + l2_square + l3_square);
  return quality;
}

// 错误函数，采用对单元度量准则取倒数的方法
double error_Function(NODE node_tp1, NODE node_tp2, NODE node_tp3)
{
  return 1 / Shape_Quality(node_tp1, node_tp2, node_tp3);
}

// 目标函数，采用优化域所有单元的错误函数相加的方法
double target_Function(std::vector<NODE> *node, NODE node_tp, std::vector<EDGE> elemNum_Optimization_Domain)
{
  double target_Function_value = 0; // 定义一个变量，储存目标函数值
  double value_tp;                  // 声明一个变量，临时储存当前判断单元错误函数值
  // 遍历 elemNum_Optimization_Domain容器，累加所有单元错误函数值
  for (std::vector<EDGE>::iterator edge_iter = elemNum_Optimization_Domain.begin(); edge_iter != elemNum_Optimization_Domain.end(); ++edge_iter)
  {
    value_tp = error_Function(node_tp, node->at(edge_iter->get_form(0)), node->at(edge_iter->get_form(1)));
    if (value_tp < 0)
      return -1;
    target_Function_value += value_tp;
  }
  return target_Function_value;
}

// 目标函数X方向上的的一阶数值导数，自变量为节点node_tp的坐标
double target_Function_X(std::vector<NODE> *node, NODE node_tp, std::vector<EDGE> elemNum_Optimization_Domain)
{
  return (target_Function(node, node_tp + node_XDelta, elemNum_Optimization_Domain) -
          target_Function(node, node_tp - node_XDelta, elemNum_Optimization_Domain)) /
         (2 * Delta);
}

// 目标函数Y方向上的的一阶数值导数，自变量为节点node_tp的坐标
double target_Function_Y(std::vector<NODE> *node, NODE node_tp, std::vector<EDGE> elemNum_Optimization_Domain)
{
  return (target_Function(node, node_tp + node_YDelta, elemNum_Optimization_Domain) -
          target_Function(node, node_tp - node_YDelta, elemNum_Optimization_Domain)) /
         (2 * Delta);
}

// 目标函数的XX二阶数值导数，自变量为节点node_tp的坐标
double target_Function_XX(std::vector<NODE> *node, NODE node_tp, std::vector<EDGE> elemNum_Optimization_Domain)
{
  return (target_Function_X(node, node_tp + node_XDelta, elemNum_Optimization_Domain) -
          target_Function_X(node, node_tp - node_XDelta, elemNum_Optimization_Domain)) /
         (2 * Delta);
}

// 目标函数的XY二阶数值导数，自变量为节点node_tp的坐标
double target_Function_XY(std::vector<NODE> *node, NODE node_tp, std::vector<EDGE> elemNum_Optimization_Domain)
{
  return (target_Function_X(node, node_tp + node_YDelta, elemNum_Optimization_Domain) -
          target_Function_X(node, node_tp - node_YDelta, elemNum_Optimization_Domain)) /
         (2 * Delta);
}

// 目标函数的YX二阶数值导数，自变量为节点node_tp的坐标
double target_Function_YX(std::vector<NODE> *node, NODE node_tp, std::vector<EDGE> elemNum_Optimization_Domain)
{
  return (target_Function_Y(node, node_tp + node_XDelta, elemNum_Optimization_Domain) -
          target_Function_Y(node, node_tp - node_XDelta, elemNum_Optimization_Domain)) /
         (2 * Delta);
}

// 目标函数的YY二阶数值导数，自变量为节点node_tp的坐标
double target_Function_YY(std::vector<NODE> *node, NODE node_tp, std::vector<EDGE> elemNum_Optimization_Domain)
{
  return (target_Function_Y(node, node_tp + node_YDelta, elemNum_Optimization_Domain) -
          target_Function_Y(node, node_tp - node_YDelta, elemNum_Optimization_Domain)) /
         (2 * Delta);
}

// 判断hessian矩阵是否正定，若不正定则修改其值使其正定，此处是使hessian矩阵加上单位矩阵的常数倍
void Positivity_Hessian_Matrix(double hessian_matrix[2][2])
{
  int Weights; // 声明一个变量，用于储存权重
  if (hessian_matrix[0][0] <= 0)
  {
    Weights = abs(hessian_matrix[0][0]) + 1;
    hessian_matrix[0][0] += Weights;
    hessian_matrix[1][1] += Weights;
  }
  if (hessian_matrix[1][1] <= 0)
  {
    Weights = abs(hessian_matrix[1][1]) + 1;
    hessian_matrix[0][0] += Weights;
    hessian_matrix[1][1] += Weights;
  }
  if (hessian_matrix[0][0] * hessian_matrix[1][1] <= hessian_matrix[0][1] * hessian_matrix[0][1])
  {
    Weights = (-(hessian_matrix[0][0] + hessian_matrix[1][1]) +
               sqrt(pow((hessian_matrix[0][0] + hessian_matrix[1][1]), 2) -
                    4 * (hessian_matrix[0][1] * hessian_matrix[0][1] - hessian_matrix[0][0] * hessian_matrix[1][1]))) /
                  2 +
              1;
    hessian_matrix[0][0] += Weights;
    hessian_matrix[1][1] += Weights;
  }
  return;
}

// 对hessian矩阵求逆，利用伴随矩阵来求
void Inverse_Hessian_Matrix(double hessian_matrix[2][2])
{
  // 定义一个变量，储存原始hessian矩阵
  double Init_hessian_matrix[2][2] = {{hessian_matrix[0][0], hessian_matrix[0][1]},
                                      {hessian_matrix[1][0], hessian_matrix[1][1]}};
  // 作为伴随矩阵前的系数，求得二维矩阵的逆矩阵
  double Weights = 1 / ((hessian_matrix[0][0] * hessian_matrix[1][1]) - (hessian_matrix[0][1] * hessian_matrix[1][0]));
  hessian_matrix[0][0] = Weights * Init_hessian_matrix[1][1];
  hessian_matrix[0][1] = -Weights * Init_hessian_matrix[0][1];
  hessian_matrix[1][0] = -Weights * Init_hessian_matrix[1][0];
  hessian_matrix[1][1] = Weights * Init_hessian_matrix[0][0];
  return;
}

// 利用牛顿法取修正的hessian逆矩阵与梯度向量乘积的负数作为牛顿方向即最速下降方向
void get_steepest_descent(double gradient_vector[2][1], double hessian_matrix[2][2], double steepest_descent[2][1])
{
  steepest_descent[0][0] = -(hessian_matrix[0][0] * gradient_vector[0][0] + hessian_matrix[0][1] * gradient_vector[1][0]);
  steepest_descent[1][0] = -(hessian_matrix[1][0] * gradient_vector[0][0] + hessian_matrix[1][1] * gradient_vector[1][0]);
  return;
}

// 构造步长辅助函数，ϕ(α) = f(xk + αdk)，a即step步长，xk是当前点位置，dk是牛顿方向，f函数即目标函数，辅助函数节点坐标加上步长后的目标函数
double step_helper_function(std::vector<NODE> *node, NODE node_tp, std::vector<EDGE> elemNum_Optimization_Domain, double steepest_descent[2][1], double step)
{
  double steepest_value[2][1]; // 声明一个变量，作为最速下降值，即步长乘上牛顿方向
  steepest_value[0][0] = step * steepest_descent[0][0];
  steepest_value[1][0] = step * steepest_descent[1][0];
  return target_Function(node, node_tp + steepest_value, elemNum_Optimization_Domain);
}

// 步长辅助函数的导数，自变量为步长step，ϕ(α)' = f'(xk + αdk)dk
double step_helper_function_prime(std::vector<NODE> *node, NODE node_tp, std::vector<EDGE> elemNum_Optimization_Domain, double steepest_descent[2][1], double step)
{
  // 步长辅助函数X与Y方向上的偏导，自变量为xk + αdk向量，分为x方向和y方向
  double step_helper_function_XY[2] = {(step_helper_function(node, node_tp + node_XDelta, elemNum_Optimization_Domain, steepest_descent, step) -
                                        step_helper_function(node, node_tp - node_XDelta, elemNum_Optimization_Domain, steepest_descent, step)) /
                                           (2 * Delta),
                                       (step_helper_function(node, node_tp + node_YDelta, elemNum_Optimization_Domain, steepest_descent, step) -
                                        step_helper_function(node, node_tp - node_YDelta, elemNum_Optimization_Domain, steepest_descent, step)) /
                                           (2 * Delta)};
  return matrix_product_1x2_2x1(step_helper_function_XY, steepest_descent);
}

// 基于修正牛顿法，利用最优步长和最速下降方向，来求每次迭代的节点坐标变化
NODE get_Node_Delta(double optimal_step, double steepest_descent[2][1])
{
  NODE node_tp; // 声明一个NODE类，临时储存节点坐标变化
  node_tp.tr_pos(0, optimal_step * steepest_descent[0][0]);
  node_tp.tr_pos(1, optimal_step * steepest_descent[1][0]);
  return node_tp;
}

// 定义一个sign函数即符号函数
double sign(double value)
{
  if (value > 0)
    return 1;
  else if (value < 0)
    return -1;
  else
    return 0;
}

// 用三次多项式插值法在区间内查找下一次迭代的步长值
double cubic_interpolation_step(std::vector<NODE> *node, NODE node_tp, std::vector<EDGE> elemNum_Optimization_Domain, double steepest_descent[2][1],
                                double step_LowerLimit, double step_UpperLimit)
{
  // 定义四个变量，分别储存区间上下限的辅助函数与辅助函数导数，作为三次多项式插值的端点条件
  double helper_function_step_Lower = step_helper_function(node, node_tp, elemNum_Optimization_Domain, steepest_descent, step_LowerLimit);
  double helper_function_step_Lower_prime = step_helper_function_prime(node, node_tp, elemNum_Optimization_Domain, steepest_descent, step_LowerLimit);
  double helper_function_step_Upper = step_helper_function(node, node_tp, elemNum_Optimization_Domain, steepest_descent, step_UpperLimit);
  double helper_function_step_Upper_prime = step_helper_function_prime(node, node_tp, elemNum_Optimization_Domain, steepest_descent, step_UpperLimit);
  // 定义两个变量，作为三次多项式插值的两个辅助值
  double d1 = helper_function_step_Lower_prime + helper_function_step_Upper_prime - 3 * (helper_function_step_Lower - helper_function_step_Upper) / (step_LowerLimit - step_UpperLimit);
  double d2 = sign(step_UpperLimit - step_LowerLimit) * sqrt(d1 * d1 - helper_function_step_Lower_prime * helper_function_step_Upper_prime);
  // 定义一个变量，储存三次多项式插值法解得的下一次迭代步长值
  double step = step_UpperLimit - (step_UpperLimit - step_LowerLimit) * (helper_function_step_Upper_prime + d2 - d1) /
                                      (helper_function_step_Upper_prime - helper_function_step_Lower_prime + 2 * d2);
  return step;
}

// 在包含最优解的区间内进一步搜索最优解
double Zoom(std::vector<NODE> *node, NODE node_tp, std::vector<EDGE> elemNum_Optimization_Domain, double steepest_descent[2][1],
            double step_LowerLimit, double step_UpperLimit)
{
  // 首先依据Zoom算法对(x,y)的要求，即ϕ(x)<ϕ(y)，来确定第一次迭代的计算区间
  double step_Low_helper_function; // 定义两个变量，作为总是满足Zoom函数条件的两个数
  double step_High_helper_function;
  double step_current;               // 声明一个变量，作为当前迭代时的步长
  double helper_function_LowStep;    // 声明一个变量，储存步长为step_Low_helper_function的辅助函数值
  double helper_function_current;    // 声明一个变量，储存本次迭代辅助函数值
  double helper_function_primeValue; // 声明一个变量，储存本次迭代辅助函数的导数值
  // 定义一个变量，储存step=0时的辅助函数值
  double helper_function_step0 = step_helper_function(node, node_tp, elemNum_Optimization_Domain, steepest_descent, 0);
  // 定义一个变量，储存step=0时的辅助函数导数值
  double helper_function_step0_prime = step_helper_function_prime(node, node_tp, elemNum_Optimization_Domain, steepest_descent, 0);
  double Armijo_right; // 声明一个变量，作为本次迭代Armijo准则的右值
  // 定义一个变量，作为强Wolfe准则的右值，该值为定值，由于牛顿方向是目标函数下降方向，此处导数必然小于0，则取负数作为强Wolfe准则的右值
  double Wolfe_right = -c2 * helper_function_step0_prime;
  if (step_helper_function(node, node_tp, elemNum_Optimization_Domain, steepest_descent, step_LowerLimit) <
      step_helper_function(node, node_tp, elemNum_Optimization_Domain, steepest_descent, step_UpperLimit))
  {
    step_Low_helper_function = step_LowerLimit;
    step_High_helper_function = step_UpperLimit;
  }
  else
  {
    step_Low_helper_function = step_UpperLimit;
    step_High_helper_function = step_LowerLimit;
  }
  // 直接进入循环，以step_current设置为满足强Wolfe条件的步长即类最优解时终止
  while (true)
  {
    // 用三次多项式插值法在区间(step_Low_helper_function,step_High_helper_function)内查找下一次迭代的步长值
    step_current = cubic_interpolation_step(node, node_tp, elemNum_Optimization_Domain, steepest_descent, step_Low_helper_function, step_High_helper_function);
    // 计算当前步长下辅助函数值
    helper_function_current = step_helper_function(node, node_tp, elemNum_Optimization_Domain, steepest_descent, step_current);
    Armijo_right = helper_function_step0 + c1 * step_current * helper_function_step0_prime;
    // 计算当前计算区间下步长为step_Low_helper_function的辅助函数值
    helper_function_LowStep = step_helper_function(node, node_tp, elemNum_Optimization_Domain, steepest_descent, step_Low_helper_function);
    // 用Armijo准则进行判断，若不满足Armijo准则或本次迭代的步长使得辅助函数值大于步长为step_Low_helper_function的辅助函数值时，调整step_High_helper_function的值，进一步缩小计算空间
    if (helper_function_current > Armijo_right + Delta || helper_function_current >= helper_function_LowStep + Delta)
      step_High_helper_function = step_current;
    else
    {
      // 计算当前步长下辅助函数的导数值
      helper_function_primeValue = step_helper_function_prime(node, node_tp, elemNum_Optimization_Domain, steepest_descent, step_current);
      // 用强Wolfe准则进行判断，若满足强Wolfe准则，说明当前步长是类最优解，直接退出循环
      if (abs(helper_function_primeValue) <= Wolfe_right + Delta)
        break;
      // 保证计算区间是连续下降区间
      if (helper_function_primeValue * (step_High_helper_function - step_Low_helper_function) >= 0)
        step_High_helper_function = step_Low_helper_function;
      // 缩小计算区间
      step_Low_helper_function = step_current;
    }
  }
  return step_current;
}

// 采用强Wolfe准则，利用牛顿方向，依据修正牛顿法可以直接取初始步长为1，确定类最优步长
double Line_Search_Algorithm_with_Wolfe(std::vector<NODE> *node, NODE node_tp, std::vector<EDGE> elemNum_Optimization_Domain,
                                        double steepest_descent[2][1], double Init_step)
{
  double step_UpperLimit = Init_step * 2; // 定义一个变量，作为每次步长迭代的区间上限，初始化为初始步长的两倍
  double step_before = 0;                 // 定义一个变量，作为上一次迭代时的步长，初始化为0
  double step_current = Init_step;        // 定义一个变量，作为当前迭代时的步长，初始化为初始步长
  // 定义一个变量，储存当前节点未加任何步长补正下优化域的目标函数值
  double all_quality_Init = target_Function(node, node_tp, elemNum_Optimization_Domain);
  // 定义一个变量，储存上次迭代后辅助函数值（即节点坐标加上步长后的目标函数值），初始化为all_quality_Init
  double helper_function_before = all_quality_Init;
  double helper_function_current;    // 声明一个变量，储存本次迭代辅助函数值
  double helper_function_primeValue; // 声明一个变量，储存本次迭代辅助函数的导数值
  // 定义一个变量，储存step=0时的辅助函数值
  double helper_function_step0 = step_helper_function(node, node_tp, elemNum_Optimization_Domain, steepest_descent, 0);
  // 定义一个变量，储存step=0时的辅助函数导数值
  double helper_function_step0_prime = step_helper_function_prime(node, node_tp, elemNum_Optimization_Domain, steepest_descent, 0);
  double Armijo_right; // 声明一个变量，作为本次迭代Armijo准则的右值
  // 定义一个变量，作为强Wolfe准则的右值，该值为定值，由于牛顿方向是目标函数下降方向，此处导数必然小于0，则取负数作为强Wolfe准则的右值
  double Wolfe_right = -c2 * helper_function_step0_prime;
  int cnt = 1; // 定义一个变量，用来记录当前迭代次数
  // 直接进入循环，以step_current设置为满足强Wolfe条件的步长即类最优解时终止
  // 本循环以查找包含最优解的区间为目标，具体类最优解的查找会调用Zoom()函数
  while (true)
  {
    // 计算当前步长下辅助函数值
    helper_function_current = step_helper_function(node, node_tp, elemNum_Optimization_Domain, steepest_descent, step_current);
    Armijo_right = helper_function_step0 + c1 * step_current * helper_function_step0_prime;
    // 用Armijo准则进行判断，若不满足Armijo准则或两次迭代后辅助函数值增加，则说明找到了包含最优解的区间，调用Zoom()函数，退出循环
    if ((helper_function_current > Armijo_right + Delta) || (helper_function_current >= helper_function_before + Delta && cnt > 1))
    {
      step_current = Zoom(node, node_tp, elemNum_Optimization_Domain, steepest_descent, step_before, step_current);
      break;
    }
    // 计算当前步长下辅助函数的导数值
    helper_function_primeValue = step_helper_function_prime(node, node_tp, elemNum_Optimization_Domain, steepest_descent, step_current);
    // 用强Wolfe准则进行判断，若满足强Wolfe准则，说明当前步长是类最优解，直接退出循环
    if (abs(helper_function_primeValue) <= Wolfe_right + Delta)
      break;
    // 如果在当前步长下辅助函数的导数值大于等于0，则说明找到了包含最优解的区间，调用Zoom()函数，退出循环
    if (helper_function_primeValue >= 0)
    {
      step_current = Zoom(node, node_tp, elemNum_Optimization_Domain, steepest_descent, step_current, step_before);
      break;
    }
    // 记录本次迭代步长
    step_before = step_current;
    // 用三次多项式插值法在区间(step_LowerLimit,step_UpperLimit)内查找下一次迭代的步长值
    step_current = cubic_interpolation_step(node, node_tp, elemNum_Optimization_Domain, steepest_descent, step_current, step_UpperLimit);
    // 记录当前迭代辅助函数值，作为下一次迭代时使用
    helper_function_before = helper_function_current;
    cnt++;
  }
  return step_current;
}

// 构造局部优化域，用有限的节点数的所有外接单元的集合来包含所有劣质单元
void Construct_Local_Optimization_Domain(std::vector<ELEM> *elem, std::vector<NODE> *node, int borderNode_num,
                                         std::vector<int> Init_inferior_elem, std::vector<int> *Laplacian_Smoothing_nodeNum)
{
  std::vector<int> Init_inferior_elem_success;        // 声明一个容器，储存判断过的初始劣质单元编号
  std::vector<int> elemNum_IncludeNode;               // 声明一个容器，用来储存包含待优化节点的所有网格单元编号
  std::vector<int>::iterator Init_inferior_elem_iter; // 声明一个迭代器，指向Init_inferior_elem容器
  int elemNum_tp;                                     // 声明一个变量，临时储存待判断的网格单元编号
  ELEM elem_tp;                                       // 声明一个ELEM变量，临时储存待判断的网格单元
  EDGE edge_tp;                                       // 声明一个EDGE变量，临时储存相邻边
  int *Adjacent_elem;                                 // 声明一个指针，用来储存当前判断单元的劣质相邻单元编号
  int cnt;                                            // 声明一个变量，记录当前判断单元的劣质相邻单元数目
  int tp;                                             // 声明一个变量，临时储存值
  bool judge;                                         // 声明一个变量，用来临时判断
  int nodeNum_tp;                                     // 声明一个变量，临时储存待判断的节点编号
  while (!Init_inferior_elem.empty())                 // 判断Init_inferior_elem内所有元素
  {
    elemNum_tp = Init_inferior_elem.front(); // 取出Init_inferior_elem中的第一个元素并擦除
    elem_tp = elem->at(elemNum_tp);
    Init_inferior_elem.erase(Init_inferior_elem.begin());
    // 首先判断该网格单元的三个节点是否都是初始网格边界点，若是，则跳过该单元
    if (elem_tp.get_form(0) < borderNode_num && elem_tp.get_form(1) < borderNode_num && elem_tp.get_form(2) < borderNode_num)
    {
      Init_inferior_elem_success.push_back(elemNum_tp);
      continue;
    }
    Adjacent_elem = new int[3]; // 开辟一个新的三个int类空间
    cnt = 0;                    // 初始化cnt
    for (int i = 0; i < DIM + 1; i++)
      if (elem_tp.get_neig(i) != -1 && elem->at(elem_tp.get_neig(i)).get_quality() < Minimum_Shape_Quality_Smoothing)
        // 判断该相邻网格单元是否判断过
        if (std::find(Init_inferior_elem_success.begin(), Init_inferior_elem_success.end(), elem_tp.get_neig(i)) ==
            Init_inferior_elem_success.end())
          *(Adjacent_elem + cnt++) = elem_tp.get_neig(i);
    // 判断cnt的大小来决定当前网格单元该生成什么样的局部优化域
    judge = false;
    switch (cnt)
    {
    case 0: // 代表当前网格单元相邻单元全是优质单元，则选取任意一个非初始网格边界点的节点作为待优化节点
    {
      for (int i = 0; i < DIM + 1; i++)
        if (elem_tp.get_form(i) >= borderNode_num)
        {
          Laplacian_Smoothing_nodeNum->push_back(elem_tp.get_form(i));
          break;
        }
      break;
    }
    case 1: // 代表只有一个相邻单元是劣质单元，则选取任意一个非初始网格边界点的节点作为待优化节点
    {
      edge_tp = elem_AdjacentEdge(elem_tp, *Adjacent_elem);
      for (int i = 0; i < DIM; i++)
        if (edge_tp.get_form(i) >= borderNode_num)
        {
          Laplacian_Smoothing_nodeNum->push_back(edge_tp.get_form(i));
          break;
        }
      break;
    }
    case 2: // 代表有两个相邻单元是劣质单元，则选取此三个劣质单元皆包含的节点作为待优化节点
    {
      tp = ThreeElem_containsNode(elem, elemNum_tp, *Adjacent_elem, *(Adjacent_elem + 1));
      if (tp == -1)
      {
        std::cout << "The function Quality_Optimization run error, the error elem number is " << elemNum_tp
                  << " , check the function." << std::endl;
        exit(-1);
      }
      else
        Laplacian_Smoothing_nodeNum->push_back(tp);
      break;
    }
    // 代表有三个相邻单元是劣质单元，则选取待判断单元与前两个相邻单元皆包含的节点作为待优化节点
    // 且选取第三个相邻单元与待判断单元相对的节点作为待优化节点（前提是该节点不是初始网格边界点，若是，则跳过）
    case 3:
    {
      tp = ThreeElem_containsNode(elem, elemNum_tp, *Adjacent_elem, *(Adjacent_elem + 1));
      if (tp == -1)
      {
        std::cout << "The function Quality_Optimization run error, the error elem number is " << elemNum_tp
                  << " , check the function." << std::endl;
        exit(-1);
      }
      else
        Laplacian_Smoothing_nodeNum->push_back(tp);
      tp = elem_tp.get_neig(AdjacentElem_pos(elem_tp, *(Adjacent_elem + 2)));
      if (tp == -1)
        judge = true;
      else
        Laplacian_Smoothing_nodeNum->push_back(tp);
      break;
    }
    default:
    {
      std::cout << "The function Quality_Optimization run error, the error elem number is " << elemNum_tp
                << " , check the function." << std::endl;
      exit(-1);
    }
    }
    // 从Init_inferior_elem内删除局部优化域内的网格单元编号，当cnt==3并且judge == false时，需要删除两个局部优化域
    for (int i = 1; i <= (cnt == 3 && judge == false ? 2 : 1); i++)
    {
      nodeNum_tp = *(Laplacian_Smoothing_nodeNum->end() - i);      // 获取最后插入的待优化节点
      std::vector<int>().swap(elemNum_IncludeNode);                // 初始化elemNum_IncludeNode，并释放容器空间
      FindBall_fast(elem, node, nodeNum_tp, &elemNum_IncludeNode); // 查找包含待优化节点的所有网格单元编号
      for (std::vector<int>::iterator iter = elemNum_IncludeNode.begin(); iter != elemNum_IncludeNode.end(); ++iter)
      {
        // 优质单元或者当前判断单元直接跳过
        if (elem->at(*iter).get_quality() >= Minimum_Shape_Quality_Smoothing || *iter == elemNum_tp)
          continue;
        Init_inferior_elem_iter = std::find(Init_inferior_elem.begin(), Init_inferior_elem.end(), *iter);
        // 没找到，代表已经判断过，跳过
        if (Init_inferior_elem_iter == Init_inferior_elem.end())
          continue;
        // 找到，则删除该单元编号
        Init_inferior_elem.erase(Init_inferior_elem_iter);
        Init_inferior_elem_success.push_back(*iter);
      }
    }
    delete[] Adjacent_elem;
  }
  return;
}

// 构造局部优化域，有劣质单元的所有节点来作为待优化节点
void Construct_Local_Optimization_Domain_All(std::vector<ELEM> *elem, std::vector<NODE> *node, int borderNode_num,
                                             std::vector<int> Init_inferior_elem, std::vector<int> *Laplacian_Smoothing_nodeNum)
{
  // 遍历Init_inferior_elem内所有单元，将非初始网格边界点全部压入Laplacian_Smoothing_nodeNum容器
  for (std::vector<int>::iterator iter = Init_inferior_elem.begin(); iter != Init_inferior_elem.end(); ++iter)
  {
    // 一个网格单元有三个节点
    for (int i = 0; i < DIM + 1; i++)
    {
      if (elem->at(*iter).get_form(i) < borderNode_num) // 跳过初始网格边界点
        continue;
      if (std::find(Laplacian_Smoothing_nodeNum->begin(), Laplacian_Smoothing_nodeNum->end(), elem->at(*iter).get_form(i)) ==
          Laplacian_Smoothing_nodeNum->end())
        Laplacian_Smoothing_nodeNum->push_back(elem->at(*iter).get_form(i));
    }
  }
  return;
}

// 智能Laplacian光顺，给定一个节点，实行Laplacian光顺，并且将该节点移动到其相邻节点的几何中心前，若不能提高相邻网格质量，则不予移动操作，移动成功返回true
bool Laplacian_Smoothing(std::vector<ELEM> *elem, std::vector<NODE> *node, int Laplacian_Smoothing_nodeNum)
{
  std::vector<int> elemNum_IncludeNode;          // 声明一个容器，用来储存包含待优化节点的所有网格单元编号
  std::vector<EDGE> elemNum_Optimization_Domain; // 创建一个容器，用来储存局部优化域的边界边
  EDGE edge_tp;                                  // 声明一个EDGE类，临时储存生成的边
  double lowest_quality_before = 1;              // 定义一个变量，储存节点优化前的局部优化域的最小单元质量，初始化为1
  double lowest_quality_after = 1;               // 定义一个变量，储存节点优化后的局部优化域的最小单元质量，初始化为1
  // 查找包含待优化节点的所有网格单元编号
  FindBall_fast(elem, node, Laplacian_Smoothing_nodeNum, &elemNum_IncludeNode);
  // 将所有局部优化域的边界边压入elemNum_Optimization_Domain，并调整边节点顺序，使得以 待优化点、边点1、边点2的顺序是三角形的逆时针顺序
  // 便于后续判断待优化点Laplacian光顺后是否有效
  for (std::vector<int>::iterator iter = elemNum_IncludeNode.begin(); iter != elemNum_IncludeNode.end(); ++iter)
  {
    InitEdge(&edge_tp); // 初始化edge_tp
    switch (ElemIncludeNode(elem->at(*iter), Laplacian_Smoothing_nodeNum))
    {
    case 0:
      edge_tp.tr_form(0, elem->at(*iter).get_form(1));
      edge_tp.tr_form(1, elem->at(*iter).get_form(2));
      break;
    case 1:
      edge_tp.tr_form(0, elem->at(*iter).get_form(0));
      edge_tp.tr_form(1, elem->at(*iter).get_form(2));
      break;
    case 2:
      edge_tp.tr_form(0, elem->at(*iter).get_form(0));
      edge_tp.tr_form(1, elem->at(*iter).get_form(1));
      break;
    default:
      break;
    }
    // 使得以 待优化点、边点1、边点2的顺序是三角形的逆时针顺序
    if (Triangle_area(node->at(Laplacian_Smoothing_nodeNum), node->at(edge_tp.get_form(0)), node->at(edge_tp.get_form(1))) < 0)
      edge_tp.Swap();
    elemNum_Optimization_Domain.push_back(edge_tp); // 压入elemNum_Optimization_Domain
    lowest_quality_before = elem->at(*iter).get_quality() < lowest_quality_before ? elem->at(*iter).get_quality() : lowest_quality_before;
  }
  // 得到Laplacian光顺后节点位置
  double x_value = 0, x_cnt = 0, y_value = 0, y_cnt = 0; // 定义一系列变量，储存x与y方向总坐标之和与总点数
  NODE node_tp = node->at(Laplacian_Smoothing_nodeNum);  // 定义一个变量，储存优化后节点，初始化为待优化点，便于储存待优化点的密度等信息
  for (std::vector<EDGE>::iterator iter = elemNum_Optimization_Domain.begin(); iter != elemNum_Optimization_Domain.end(); ++iter)
    // 一个局部优化域的边界边有两个节点
    for (int i = 0; i < DIM; i++)
    {
      x_value += node->at(iter->get_form(i)).get_pos(0);
      x_cnt++;
      y_value += node->at(iter->get_form(i)).get_pos(1);
      y_cnt++;
    }
  node_tp.tr_pos(0, x_value / x_cnt); // Laplacian光顺后节点x坐标
  node_tp.tr_pos(1, y_value / y_cnt); // Laplacian光顺后节点y坐标
  // 计算优化后节点与各局部优化域的边界边形成的三角形质量，若有小于0的值，说明该优化点位置无效，直接返回false，并且记录优化后的局部优化域的最小单元质量
  double quality_tp; // 定义一个变量，储存临时单元质量
  for (std::vector<EDGE>::iterator iter = elemNum_Optimization_Domain.begin(); iter != elemNum_Optimization_Domain.end(); ++iter)
  {
    quality_tp = Shape_Quality(node_tp, node->at(iter->get_form(0)), node->at(iter->get_form(1)));
    // 如果当前三角形质量小于0，说明该优化点位置无效，直接返回false
    if (quality_tp < 0)
      return false;
    // 记录优化后的局部优化域的最小单元质量
    lowest_quality_after = quality_tp < lowest_quality_after ? quality_tp : lowest_quality_after;
  }
  // 如果优化后的局部优化域的最小单元质量大于优化前的局部优化域的最小单元质量，则确定优化成功，修改待优化点坐标，并且修改局部优化域内每一个网格单元的单元质量
  if (lowest_quality_after > lowest_quality_before)
  {
    node->at(Laplacian_Smoothing_nodeNum) = node_tp;
    // 修改局部优化域内每一个网格单元的单元质量
    for (std::vector<int>::iterator iter = elemNum_IncludeNode.begin(); iter != elemNum_IncludeNode.end(); ++iter)
      elem->at(*iter).tr_quality(abs(Shape_Quality(node->at(elem->at(*iter).get_form(0)),
                                                   node->at(elem->at(*iter).get_form(1)),
                                                   node->at(elem->at(*iter).get_form(2)))));
    return true;
  }
  else
    return false;
}

// 基于优化的光顺，给定一个节点，采用对目标函数求最优解的方法确定新节点位置，目标函数为该节点相邻单元中最差单元的单元质量函数的倒数或者（1-网格质量函数），移动成功返回true
bool Optimization_based_Smoothing_The_worst()
{
  return false;
}

// 基于优化的光顺，给定一个节点，采用对目标函数求最优解的方法确定新节点位置，目标函数为该节点相邻单元的所有质量函数的倒数相加，即优化域错误函数相加，移动成功返回true
bool Optimization_based_Smoothing_The_all(std::vector<ELEM> *elem, std::vector<NODE> *node, int Optimization_based_Smoothing_nodeNum)
{
  double all_quality_before = 0; // 定义一个变量，储存上次迭代后目标函数值，初始化为0
  double all_quality_after = 0;  // 定义一个变量，储存本次迭代后目标函数值，初始化为0
  // * 首先得到局部优化域的初始目标函数值
  std::vector<int> elemNum_IncludeNode; // 声明一个容器，用来储存包含待优化节点的所有网格单元编号
  // 查找包含待优化节点的所有网格单元编号
  FindBall_fast(elem, node, Optimization_based_Smoothing_nodeNum, &elemNum_IncludeNode);
  // 目标函数为该节点相邻单元的所有错误函数相加，此处直接累加所有单元的质量倒数即可
  for (std::vector<int>::iterator iter = elemNum_IncludeNode.begin(); iter != elemNum_IncludeNode.end(); ++iter)
    all_quality_before += 1 / elem->at(*iter).get_quality();
  // * 储存局部优化域的边界边
  std::vector<EDGE> elemNum_Optimization_Domain; // 创建一个容器，用来储存局部优化域的边界边
  EDGE edge_tp;                                  // 声明一个EDGE类，临时储存生成的边
  // 将所有局部优化域的边界边压入elemNum_Optimization_Domain，并调整边节点顺序，使得以 待优化点、边点1、边点2的顺序是三角形的逆时针顺序
  // 便于后续判断待优化点Optimization_based光顺后是否有效
  for (std::vector<int>::iterator iter = elemNum_IncludeNode.begin(); iter != elemNum_IncludeNode.end(); ++iter)
  {
    InitEdge(&edge_tp); // 初始化edge_tp
    switch (ElemIncludeNode(elem->at(*iter), Optimization_based_Smoothing_nodeNum))
    {
    case 0:
      edge_tp.tr_form(0, elem->at(*iter).get_form(1));
      edge_tp.tr_form(1, elem->at(*iter).get_form(2));
      break;
    case 1:
      edge_tp.tr_form(0, elem->at(*iter).get_form(0));
      edge_tp.tr_form(1, elem->at(*iter).get_form(2));
      break;
    case 2:
      edge_tp.tr_form(0, elem->at(*iter).get_form(0));
      edge_tp.tr_form(1, elem->at(*iter).get_form(1));
      break;
    default:
      break;
    }
    // 使得以 待优化点、边点1、边点2的顺序是三角形的逆时针顺序
    if (Triangle_area(node->at(Optimization_based_Smoothing_nodeNum), node->at(edge_tp.get_form(0)), node->at(edge_tp.get_form(1))) < 0)
      edge_tp.Swap();
    elemNum_Optimization_Domain.push_back(edge_tp); // 压入elemNum_Optimization_Domain
  }
  NODE node_original = node->at(Optimization_based_Smoothing_nodeNum); // 定义一个NODE类，作为原始点，即程序输入的待优化点
  NODE node_before = node_original;                                    // 声明一个NODE类，储存本次迭代前点位置，初始化为原始点位置
  NODE node_after;                                                     // 声明一个NODE类，储存本次迭代后点位置
  NODE node_Delta;                                                     // 声明一个NODE类，作为每次迭代的节点坐标变化值
  double gradient_vector[2][1];                                        // 声明一个变量，作为梯度向量，也即当前位置目标函数最快下降方向
  double hessian_matrix[2][2];                                         // 声明一个变量，作为hessian矩阵
  // 声明一个变量，作为当前迭代位置的最速下降方向，利用牛顿法取修正的hessian逆矩阵与梯度向量乘积的负数作为牛顿方向即最速下降方向
  double steepest_descent[2][1];
  // gradient_vector[0][0] = target_Function_X(node, node_before, elemNum_Optimization_Domain);
  // gradient_vector[1][0] = target_Function_Y(node, node_before, elemNum_Optimization_Domain);
  double optimal_step; //= 0.01 / sqrt(pow(gradient_vector[0][0], 2) + pow(gradient_vector[1][0], 2)); // 定义一个变量，作为每次迭代的类最优步长
  int iter = 0;        // 声明一个变量，作为迭代次数
  // * 利用带线搜索的修正牛顿法来求每次迭代的节点坐标变化
  do
  {
    // 储存上次迭代结束后目标函数值，第一次迭代时，不需要储存
    if (all_quality_after != 0)
      all_quality_before = all_quality_after;
    // 先由当前点位置求目标函数当前位置梯度
    gradient_vector[0][0] = target_Function_X(node, node_before, elemNum_Optimization_Domain);
    gradient_vector[1][0] = target_Function_Y(node, node_before, elemNum_Optimization_Domain);
    // 梯度向量的模小于最小梯度值时代表达到极值点处，停止迭代
    if (sqrt(pow(gradient_vector[0][0], 2) + pow(gradient_vector[1][0], 2)) < Min_gradient)
      break;
    // 求当前点位置目标函数的hessian矩阵
    hessian_matrix[0][0] = target_Function_XX(node, node_before, elemNum_Optimization_Domain);
    hessian_matrix[0][1] = target_Function_XY(node, node_before, elemNum_Optimization_Domain);
    hessian_matrix[1][0] = target_Function_YX(node, node_before, elemNum_Optimization_Domain);
    hessian_matrix[1][1] = target_Function_YY(node, node_before, elemNum_Optimization_Domain);
    // 判断hessian矩阵是否正定，若不正定则修改其值使其正定
    Positivity_Hessian_Matrix(hessian_matrix);
    // 对hessian矩阵矩阵求逆
    Inverse_Hessian_Matrix(hessian_matrix);
    // steepest_descent[0][0] = -gradient_vector[0][0];
    // steepest_descent[1][0] = -gradient_vector[1][0];
    //  求牛顿方向即当前位置最速下降方向，即负的hessian逆矩阵乘梯度
    get_steepest_descent(gradient_vector, hessian_matrix, steepest_descent);
    // 采用强Wolfe准则，利用牛顿方向，依据修正牛顿法可以直接取初始步长为1，确定类最优步长
    optimal_step = Line_Search_Algorithm_with_Wolfe(node, node_before, elemNum_Optimization_Domain, steepest_descent, 1);
    // 求本次迭代节点坐标变化
    node_Delta = get_Node_Delta(optimal_step, steepest_descent);
    node_after = node_before + node_Delta;
    all_quality_after = target_Function(node, node_after, elemNum_Optimization_Domain);
    node_before = node_after;
    // optimal_step /= 2;
    iter++;
  } while (optimal_step > Min_step && iter < Max_iter && abs(all_quality_after - all_quality_before) > Min_imp);
  node->at(Optimization_based_Smoothing_nodeNum) = node_before;
  // 修改局部优化域内每一个网格单元的单元质量
  for (std::vector<int>::iterator iter = elemNum_IncludeNode.begin(); iter != elemNum_IncludeNode.end(); ++iter)
    elem->at(*iter).tr_quality(abs(Shape_Quality(node->at(elem->at(*iter).get_form(0)),
                                                 node->at(elem->at(*iter).get_form(1)),
                                                 node->at(elem->at(*iter).get_form(2)))));
  return true;
}

/*
 * 1
 */

// 读取输入文件
void ReadInFile(std::vector<ELEM> *elem, int *elem_num, std::vector<NODE> *node, int *node_num, int *borderNode_num,
                int *borderType, double *density_control, std::string infileName)
{
  std::fstream infile; // 声明输入文件流
  std::string filePath = "MESH\\" + infileName;
  infile.open(filePath, std::ios::in);                     // 打开infileName，并且只能进行读取操作
  std::cout << "Reading from the file......" << std::endl; // 正在读取文件
  if (!infile.is_open())
  {                                                           // 判断文件是否存在或者是否成功打开
    std::cout << "error on open " << infileName << std::endl; // 文件不能成功打开
    system("pause");
    exit(-1);
  }
  else
  {
    std::cout << "Reading successfully!" << std::endl; // 读取文件成功
    infile >> *node_num;                               // 读入总节点数总节点数
    *borderNode_num = *node_num;                       // 储存总节点数目
    // 读入边界点类型
    do
      infile >> *borderType;
    while (*(borderType++) != -1);
    // 读入密度控制信息
    double tp1;
    do
    {
      infile >> tp1;
      *density_control++ = tp1;
    } while (tp1 != -1);
    double node_t[] = {0, 0, 0}; // 声明临时数组node_t用于接收输入数据，便于赋值到NODE类
    NODE node_tp;                // 声明临时NODE类用于压入容器
    int tp;                      // 声明用于临时存放值的变量
    for (int i = 0; i < *node_num; i++)
    {
      memset(node_t, 0, sizeof(node_t)); // 初始化node_t
      InitNode(&node_tp);                // 初始化node_tp
      infile >> tp;
      infile >> node_t[0] >> node_t[1] >> node_t[2]; // 读入节点坐标
      for (int j = 0; j < DIM; j++)
        node_tp.tr_pos(j, node_t[j]); // 节点坐标赋值到node_tp
      node_tp.tr_spac(node_t[2]);     // 节点密度信息赋值到node_tp
      node->push_back(node_tp);       // 压入容器node
    }
    infile >> *elem_num; // 读取输入网格单元数
    if (!*elem_num)      // 如果没有输入网格单元，关闭文件退出该程序
    {
      infile.close();
      return;
    }
  }
  infile.close();
  return;
}

// 输出生成网格文件
void OutputFile(std::vector<ELEM> *elem, int *elem_num, std::vector<NODE> *node, int *node_num, std::string outfileName)
{
  std::fstream outfile; // 声明输出文件流
  std::string filePath = "MESH\\" + outfileName;
  outfile.open(filePath, std::ios::out | std::ios::trunc); // 创建或者打开输出文件，如果此文件已经存在, 则打开文件之前把文件长度截断为0
  if (!outfile.is_open())
  {                                                            // 判断文件是是否成功打开
    std::cout << "error on open " << outfileName << std::endl; // 文件不能成功打开
    system("pause");
    exit(-1);
  }
  else // 以tetview网格文件方式输出网格信息
  {
    // 输出节点信息
    outfile << *node_num << ' ' << DIM + 1 << ' ' << 0 << ' ' << 0 << '\n';
    int node_cnt = 0; // 定义节点输出标号变量，第一个节点标号显然为零
    // 使用迭代器遍历node，输出节点信息
    for (std::vector<NODE>::iterator node_iter = node->begin(); node_iter != node->end(); ++node_iter)
    {
      outfile << node_cnt << ' ' << node_iter->get_pos(0) << ' ' << node_iter->get_pos(1) << " 0" << '\n';
      node_cnt += 1; // 每成功输出一个节点，node_cnt++
    }
    // 输出网格单元信息
    outfile << *elem_num << ' ' << 0 << '\n';
    int elem_NodeNum = 3; // 定义网格单元节点数，显然Delaunay三角化生成的网格单元是三角形有3个节点
    // 使用迭代器遍历elem，输出网格单元信息
    for (std::vector<ELEM>::iterator elem_iter = elem->begin(); elem_iter != elem->end(); ++elem_iter)
      outfile << elem_NodeNum << ' ' << elem_iter->get_form(0) << ' ' << elem_iter->get_form(1) << ' ' << elem_iter->get_form(2) << '\n';
    outfile << 0 << '\n'
            << 0;                                                      // 以tetview网格文件方式输出网格信息
    std::cout << outfileName << " outing successfully !" << std::endl; // 文件输出成功
  }
  outfile.close();
  return;
}

// 生成边界边
void Gen_BorderEdge(int node_num, int *borderType, std::vector<EDGE> *border_edge, int *border_edge_num)
{
  int type;         // 声明一个变量，用于判断边界点形成图形的类型
  int border_num;   // 声明一个变量，用来储存type类型图形的边界点数量
  int node_cnt = 0; // 声明一个变量，用来储存当前总处理过的边界点数
  EDGE edge_tp;     // 声明一个用于临时储存生成边界边的edge类
  // 依据borderType内信息生成边界边
  do
  {
    type = *borderType++;
    border_num = *borderType++;
    // 如果是内部点，则跳过
    if (type == 5)
      continue;
    for (int i = node_cnt; i < border_num + node_cnt - 1; i++)
    {
      InitEdge(&edge_tp); // 初始化edge_tp
      edge_tp.tr_form(0, i);
      edge_tp.tr_form(1, i + 1);
      border_edge->push_back(edge_tp); // 将当前生成的边界边压入border_edge中，形成边界边生成
      *border_edge_num += 1;
    }
    InitEdge(&edge_tp);           // 初始化edge_tp
    edge_tp.tr_form(0, node_cnt); // 最后一个节点是与最前一个节点形成的边
    edge_tp.tr_form(1, border_num + node_cnt - 1);
    border_edge->push_back(edge_tp); // 将当前生成的边界边压入border_edge中，形成边界边生成
    *border_edge_num += 1;
    node_cnt += border_num;
  } while (*borderType != -1); //-1是*borderType的结尾
  return;
}

// 生成初始Delaunay三角化，插入初始Delaunay三角化四边形边框的四个顶角节点，输出该三角化信息文件 Ini_Delaunay.smesh
void Output_IniDeDelaunay(std::vector<ELEM> *elem, int *elem_num, std::vector<NODE> *node, int *node_num, int *borderType, int DeFrame_numPos[])
{
  // 生成初始Delaunay三角化四边形边框，并插入四个顶角节点
  Gen_IniDeFrame(node, node_num, DeFrame_numPos);
  // 生成初始Delaunay三角化
  Gen_IniDeDelaunay(elem, elem_num, node, DeFrame_numPos);
  std::fstream outfile; // 声明输出文件流
  // 创建或者打开Ini_Delaunay.smesh，如果此文件已经存在, 则打开文件之前把文件长度截断为0
  outfile.open("MESH\\Ini_Delaunay.smesh", std::ios::out | std::ios::trunc);
  if (!outfile.is_open())
  {                                                  // 判断文件是是否成功打开
    std::cout << "error on open Ini_Delaunay.smesh"; // 文件不能成功打开
    system("pause");
    exit(-1);
  }
  // 以tetview网格文件方式输出网格信息
  else
  {
    // 输出节点信息
    outfile << *node_num << ' ' << DIM + 1 << ' ' << 0 << ' ' << 0 << '\n';
    int node_cnt = 0; // 定义节点输出标号变量，第一个节点标号显然为零
    // 使用迭代器遍历node，输出节点信息
    for (std::vector<NODE>::iterator node_iter = node->begin(); node_iter != node->end(); ++node_iter)
    {
      outfile << node_cnt << ' ' << node_iter->get_pos(0) << ' ' << node_iter->get_pos(1) << " 0" << '\n';
      node_cnt += 1; // 每成功输出一个节点，node_cnt++
    }
    // 输出网格单元信息
    // 首先获取原始输入图形数量
    int cnt = 0;
    while (*(borderType + cnt) != -1)
      cnt++;
    cnt = cnt / 2;
    outfile << (*elem_num) + cnt << ' ' << 0 << '\n'; // 包含输入图形单元
    int type;                                         // 声明一个变量，用于判断边界点形成图形的类型
    int border_num;                                   // 声明一个变量，用来储存type类型图形的边界点数量
    int nodeDeal_cnt = 0;                             // 声明一个变量，用来储存当前总处理过的边界点数
    // 依据borderType内信息生成原始输入图形
    do
    {
      type = *borderType++;
      border_num = *borderType++;
      outfile << border_num; // 当前图形节点数
      for (int i = nodeDeal_cnt; i < border_num + nodeDeal_cnt; i++)
        outfile << ' ' << i;
      outfile << '\n';
      nodeDeal_cnt += border_num;
    } while (*borderType != -1); //-1是*borderType的结尾
    int elem_NodeNum = 3;        // 定义网格单元节点数，显然Delaunay三角化生成的网格单元是三角形有3个节点
    // 使用迭代器遍历elem，输出网格单元信息
    for (std::vector<ELEM>::iterator elem_iter = elem->begin(); elem_iter != elem->end(); ++elem_iter)
      outfile << elem_NodeNum << ' ' << elem_iter->get_form(0) << ' ' << elem_iter->get_form(1) << ' ' << elem_iter->get_form(2) << '\n';
    outfile << 0 << '\n'; // 以tetview网格文件方式输出网格信息
    outfile << 0;
    std::cout << "Ini_Delaunay.smesh outing successfully!" << std::endl; // 文件输出成功
  }
  outfile.close();
  return;
}

// 边界点插入
void InsertBoundaryPoint(std::vector<ELEM> *elem, int *elem_num, std::vector<NODE> *node, int borderNode_num)
{
  ELEM elem_tp;                            // 声明一个用于临时储存生成网格单元的变量
  NODE node_Insert;                        // 声明一个用于临时储存待插入边界点的变量
  EDGE edge_tp;                            // 声明一个用于临时储存生成的边的变量
  for (int k = 0; k < borderNode_num; k++) // 一个个插入边界点
  {
    InitNode(&node_Insert);    // 初始化node_Insert
    node_Insert = node->at(k); // 一个个插入边界点
    // 声明一个变量，用来储存任意一个外接圆包含待插入节点的网格单元编号，作为基单元来查找空腔
    // 为简化搜索流程，从包含上次插入的边界点的网格单元开始搜索
    int elemNum_IncludeNode = FindElemIncludeNode(elem, node, node_Insert, k == 0 ? 0 : node->at(k - 1).get_elem());
    // 创建一个空腔容器，用来储存外接圆包含待插入节点node_Insert的网格单元编号
    std::vector<int> elemNum_Cav;
    elemNum_Cav.push_back(elemNum_IncludeNode);                                                         // 压入直接包含待插入节点的网格单元编号
    FindElemCav(elem, elem_num, node, &borderNode_num, &elemNum_Cav, node_Insert, elemNum_IncludeNode); // 空腔查找
    // RepairCavity(elem, elem_num, node, node_num, &elemNum_Cav, node_Insert);//空腔修复
    std::vector<EDGE> edge_Cav; // 创建一个容器，用来储存空腔边界，一个边由两个节点构成
    /**
     * * 在edge_Cav内查找当前搜索的网格单元边界，如果搜到end()，代表没有找到，该边是空腔边，可以插入；
     * * 如果没有搜到搜到end()，代表该边不是空腔边，不予插入，并且在edge_Cav内删除该边
     */
    // 将所有空腔边界压入edge_Cav
    for (unsigned long long i = 0; i < elemNum_Cav.size(); i++)
    {
      InitEdge(&edge_tp);                 // 初始化edge_tp
      int elemNum_tp = elemNum_Cav.at(i); // 取出空腔内一个网格单元
      // 定义一个变量，储存网格单元所有边，便于后续对边的访问
      int edge_Cavtp[3][2] = {{elem->at(elemNum_tp).get_form(0), elem->at(elemNum_tp).get_form(1)},  // 显然一个网格单元有三条边
                              {elem->at(elemNum_tp).get_form(1), elem->at(elemNum_tp).get_form(2)},  // 取节点编号的时候从小到大排列
                              {elem->at(elemNum_tp).get_form(0), elem->at(elemNum_tp).get_form(2)}}; // 便于后边各种查找
      // 一个网格单元有三条边
      for (int j = 0; j < 3; j++)
      {
        InitEdge(&edge_tp); // 初始化edge_tp
        edge_tp.tr_form(0, edge_Cavtp[j][0]);
        edge_tp.tr_form(1, edge_Cavtp[j][1]);
        std::vector<EDGE>::iterator edge_Iter; // 创建一个迭代器，便于删除非空腔边界边
        if ((edge_Iter = std::find(edge_Cav.begin(), edge_Cav.end(), edge_tp)) == edge_Cav.end())
          edge_Cav.push_back(edge_tp);
        else
          edge_Cav.erase(edge_Iter); // 显然，该边被插入过elemNum_edge，说明该边不是空腔边界边，删除边
      }
    }
    InitElemCav(elem, &elemNum_Cav); // 空腔初始化操作，在elem内中初始化空腔所包含的网格单元
    // 对每个空腔边界进行操作，生成待插入点与当前空腔边界形成的网格单元，首先更新空腔边界边的相邻关系
    for (unsigned long long i = 0; i < edge_Cav.size(); i++)
    {
      // 前elemNum_Cav.size()个边与待插入点生成的网格单元编号采用空腔中删除的网格单元编号
      if (i < elemNum_Cav.size())
      {
        int elemNum_tp = elemNum_Cav.at(i); // 取出空腔内一个网格单元编号
        // 显然可以直接给新生成的网格单元赋节点编号
        elem->at(elemNum_tp).tr_form(0, k);
        elem->at(elemNum_tp).tr_form(1, edge_Cav.at(i).get_form(0));
        elem->at(elemNum_tp).tr_form(2, edge_Cav.at(i).get_form(1));
        // 交换网格单元节点信息form数组，使其从小到大排列
        elem->at(elemNum_tp).Sort();
        // 改变该网格单元的三个节点的elem值，使其全部指向该单元
        node->at(k).tr_elem(elemNum_tp);
        node->at(edge_Cav.at(i).get_form(0)).tr_elem(elemNum_tp);
        node->at(edge_Cav.at(i).get_form(1)).tr_elem(elemNum_tp);
        Update_Djacency(elem, node, edge_Cav.at(i), "slow"); // 更新该边相邻关系
      }
      // 之后生成的网格全压入elem中，并赋予新的编号
      else
      {
        InitElem(&elem_tp); // 初始化elem_tp
        // 显然可以直接给新生成的网格单元赋节点编号
        elem_tp.tr_form(0, k);
        elem_tp.tr_form(1, edge_Cav.at(i).get_form(0));
        elem_tp.tr_form(2, edge_Cav.at(i).get_form(1));
        // 交换网格单元节点信息form数组，使其从小到大排列
        elem_tp.Sort();
        // 改变该网格单元的三个节点的elem值，使其全部指向该单元
        node->at(k).tr_elem(*elem_num);
        node->at(edge_Cav.at(i).get_form(0)).tr_elem(*elem_num);
        node->at(edge_Cav.at(i).get_form(1)).tr_elem(*elem_num);
        elem->push_back(elem_tp); // 将elem_tp1压入elem中，形成网格单元的生成
        *elem_num += 1;
        Update_Djacency(elem, node, edge_Cav.at(i), "slow"); // 查找包含该边的所有网格单元，并更新相邻关系
      }
    }
    std::vector<int> elemNum_edgeJudge; // 创建一个容器，用来储存检查过的空腔边节点
    // 更新插入点与空腔边节点连接形成的边的相邻关系
    for (unsigned long long i = 0; i < edge_Cav.size(); i++)
    {
      // 注意，一个边有2个节点
      // 节点一
      if (find(elemNum_edgeJudge.begin(), elemNum_edgeJudge.end(), edge_Cav.at(i).get_form(0)) == elemNum_edgeJudge.end())
      {
        // 显然，该节点没有被检查过，插入节点，更新该节点形成边的相邻关系
        InitEdge(&edge_tp);
        // 将待插入节点的编号与当前操作节点编号做比较，使其在数组内从小到大排列，便于检索边界
        edge_tp.tr_form(0, std::min(k, edge_Cav.at(i).get_form(0)));
        edge_tp.tr_form(1, std::max(k, edge_Cav.at(i).get_form(0)));
        Update_Djacency(elem, node, edge_tp, "slow");            // 查找包含该边的所有网格单元，并更新相邻关系
        elemNum_edgeJudge.push_back(edge_Cav.at(i).get_form(0)); // 插入该节点编号到elemNum_edgeJudge
        if (elemNum_edgeJudge.size() == edge_Cav.size())         // 显然只会形成与空腔边界边数量相同的边数量，达到该数量则直接退出循环
          break;
      }
      // 节点二
      if (find(elemNum_edgeJudge.begin(), elemNum_edgeJudge.end(), edge_Cav.at(i).get_form(1)) == elemNum_edgeJudge.end())
      {
        // 显然，该节点没有被检查过，插入节点，更新该节点形成边的相邻关系
        InitEdge(&edge_tp);
        // 将待插入节点的编号与当前操作节点编号做比较，使其在数组内从小到大排列，便于检索边界
        edge_tp.tr_form(0, std::min(k, edge_Cav.at(i).get_form(1)));
        edge_tp.tr_form(1, std::max(k, edge_Cav.at(i).get_form(1)));
        Update_Djacency(elem, node, edge_tp, "slow");            // 查找包含该边的所有网格单元，并更新相邻关系
        elemNum_edgeJudge.push_back(edge_Cav.at(i).get_form(1)); // 插入该节点编号到elemNum_edgeJudge
        if (elemNum_edgeJudge.size() == edge_Cav.size())         // 显然只会形成与空腔边界边数量相同的边数量，达到该数量则直接退出循环
          break;
      }
    }
  }
  return;
}

// 搜索初始Delaunay三角化方形边框的四个顶角节点与边界点形成的单元，并打上标识
void Search_DeFrame_Grid(std::vector<ELEM> *elem, int elem_num, int DeFrame_numPos[])
{
  for (std::vector<ELEM>::iterator elem_iter = elem->begin(); elem_iter != elem->end(); ++elem_iter)
  {
    // 由于初始Delaunay三角化方形边框的四个顶角节点标号总是最大，所以直接查找每个单元的最大节点标号即可
    if (elem_iter->get_form(2) == DeFrame_numPos[0] ||
        elem_iter->get_form(2) == DeFrame_numPos[1] ||
        elem_iter->get_form(2) == DeFrame_numPos[2] ||
        elem_iter->get_form(2) == DeFrame_numPos[3])
      elem_iter->tr_DeFrame_Grid(true);
  }
  return;
}

// 内部点生成和插入
void AutoRefine(std::vector<ELEM> *elem, int *elem_num, std::vector<NODE> *node, int *node_num, double *density_control)
{
  std::vector<NODE> node_Insert; // 创建储存待插入节点的容器
  // 创建一个容器，用来储存生成插入点时直接包含待插入节点的网格单元编号，标号与node_Insert一一对应
  // 该容器作为后续网格变化后再次查找包含某插入节点的网格提供便利，能优化查找时间
  std::vector<int> elemNum_IncludeNodeInitial;
  std::vector<NODE> node_InsSucc; // 创建一个容器，用来储存成功插入的节点
  // int cnt = 1;
  do
  {
    std::vector<NODE>().swap(node_Insert);               // 初始化node_Insert，并释放容器空间
    std::vector<int>().swap(elemNum_IncludeNodeInitial); // 初始化elemNum_IncludeNodeInitial，并释放容器空间
    std::vector<NODE>().swap(node_InsSucc);              // 初始化node_InsSucc，并释放容器空间
    // 内部点生成
    CreateFieldNodes(elem, elem_num, node, node_num, &node_Insert, density_control, &elemNum_IncludeNodeInitial);
    bool Sign;                // 判断节点是否成功插入
    if (!node_Insert.empty()) // 如果待插入节点容器不为空，则开始插入节点
    {
      for (unsigned long long i = 0; i < node_Insert.size(); i++)
      {
        // 声明一个变量，用来储存任意一个外接圆包含待插入节点的网格单元编号，作为基单元来查找空腔
        int elemNum_IncludeNodeIns;
        // 查找一个外接圆包含待插入节点的网格单元编号
        elemNum_IncludeNodeIns = FindElemIncludeNode(elem, node, node_Insert.at(i), elemNum_IncludeNodeInitial.at(i));
        // 如果该值为-1，说明程序运行错误
        if (elemNum_IncludeNodeIns == -1)
        {
          std::cout << "Error file output:" << std::endl;
          OutputFile(elem, elem_num, node, node_num, "Error_file");
          exit(-1);
        }
        // 插入该节点，并得到节点是否成功插入的信息
        Sign = InsertFieldPoint(elem, elem_num, node, node_num, node_Insert.at(i), elemNum_IncludeNodeIns);
        if (Sign)
        {
          node_InsSucc.push_back(node_Insert.at(i));  // 如果节点成功插入，则将成功插入的节点放入容器node_InsSucc
          node->push_back(node_Insert.at(i));         // 将成功插入的节点放入容器node中，形成节点插入
          node->at(*node_num).tr_elem(*elem_num - 1); // 最新生成的网格单元一定包含最新插入的节点
          *node_num += 1;                             // 每插入一个节点记一次数
        }
      }
    }
    // std::string str = ".smesh";
    // str = std::to_string(cnt) + str;
    // OutputFile(elem, elem_num, node, node_num, str); // 输出网格信息文件 Final_Delaunay.smesh
    // cnt++;
  } while (!node_InsSucc.empty()); // 如果该次循环成功插入了节点，则再执行一次循环
  return;
}

// 通过对角交换实现约束边界恢复
void Constrained_BoundaryRecovery(std::vector<ELEM> *elem, std::vector<NODE> *node, std::vector<EDGE> *border_edge, int *border_edge_num)
{
  std::vector<int> elemNum_IncludeEdge; // 声明一个容器，荣来储存包含*edge_iter边的网格单元编号
  // 检索每个边界边，看当前Delaunay三角化网格单元是否包含该边界边
  for (std::vector<EDGE>::iterator edge_iter = border_edge->begin(); edge_iter != border_edge->end(); ++edge_iter)
  {
    std::vector<int>().swap(elemNum_IncludeEdge);                    // 初始化elemNum_IncludeEdge，并释放容器空间
    FindShell(elem, node, *edge_iter, &elemNum_IncludeEdge, "fast"); // 查找包含*edge_iter边的网格单元编号
    // 如果elemNum_IncludeEdge容器大小为零，则代表当前查找的边界边不存在于当前三角化内，需要边界恢复
    if (elemNum_IncludeEdge.size() == 0)
    {
      std::cout << "The boundary formed by node " << edge_iter->get_form(0) << " and "
                << edge_iter->get_form(1) << " is not in the current triangulation!" << std::endl;
      std::cout << "Restoring boundary......" << std::endl;
      // 通过不断对角交换实现边界恢复
      Swap_BoundaryRecovery(elem, node, *edge_iter);
      std::cout << "Boundary restored successfully!" << std::endl;
    }
  }
  return;
}

// 去掉外部单元并缩减容器
void Removal_ExGrid(std::vector<ELEM> *elem, int *elem_num, std::vector<NODE> *node, int *node_num, int DeFrame_numPos[])
{
  int *nodeNum_Remove = NULL;      // 声明一个指针，用来指向储存待移除节点的连续内存空间
  std::vector<int> elemNum_Remove; // 声明一个容器，用来储存待移除网格单元编号
  // int *elemNum_Remove = NULL; // 声明一个指针，用来指向储存待移除网格单元的连续内存空间
  // nodeNum_Remove = Node_NotInProblemDomain();//查找所有没在问题域内的网格节点
  // elemNum_Remove = Elem_NotInProblemDomain(); // 查找所有没在问题域内的网格单元
  // 此处nodeNum_Remove直接指向初始Delaunay三角化方形边框的四个顶角节点数组就行
  nodeNum_Remove = DeFrame_numPos;
  // elemNum_Remove=new int;//开辟一个新int空间
  for (int i = 0; i < *elem_num; i++)
    if (elem->at(i).get_DeFrame_Grid())
      elemNum_Remove.push_back(i);
  Removal_NodeElem(elem, elem_num, node, node_num, &elemNum_Remove, nodeNum_Remove); // 在node容器内删除节点与单元并更新所有信息
  // Removal_Elem(elem, elem_num, node, node_num, elemNum_Remove); // 在elem容器内删除单元并更新所有信息
  return;
}

// 检查elem类中相邻信息的准确性
void Check_ElemAdjacency_accuracy(std::vector<ELEM> *elem, int *elem_num, std::vector<NODE> *node, int *node_num)
{
  if (elem->size() == 0)
    return;
  if (elem->size() != *elem_num)
    std::cout << "elem_num is not true!" << std::endl;
  // 检索每个网格单元
  ELEM elem_tp;
  for (int i = 0; i < *elem_num; i++)
  {
    InitElem(&elem_tp);
    elem_tp = elem->at(i);
    for (int j = 0; j < DIM + 1; j++)
    {
      if (elem_tp.get_neig(j) != -1)
      {
        if (elem_tp.get_neig(j) > *elem_num)
          std::cout << "the value of the elem_num is wrong" << std::endl;
        else if (!Elem_Adjacent(elem->at(elem_tp.get_neig(j)), elem->at(i)))
          std::cout << "The value of the " << i << " elem is wrong" << std::endl;
      }
    }
  }
  return;
}

// 检查node类中elem的准确性
void Check_NodeElem_accuracy(std::vector<ELEM> *elem, int *elem_num, std::vector<NODE> *node, int *node_num)
{
  if (node->size() != *node_num)
    std::cout << "node_num is not true!" << std::endl;
  // 检索每个节点，看每个节点的elem是否准确
  for (int i = 0; i < *node_num; i++)
  {
    int elem_Numtp = node->at(i).get_elem(); // 拿出第i个节点的elem值，即第elem_Numtp个网格单元
    if (elem_Numtp >= *elem_num)
      std::cout << "the value of the elem_num  is wrong" << std::endl;
    else if (elem_Numtp == -1)
      continue;
    else if (elem->at(elem_Numtp).get_form(0) == i ||
             elem->at(elem_Numtp).get_form(1) == i ||
             elem->at(elem_Numtp).get_form(2) == i)
      continue;
    std::cout << "The value of the " << i << " node is wrong" << std::endl;
  }
  return;
}

// 计算各初始网格单元质量并保存
void Calculate_Shape_Quality(std::vector<ELEM> *elem, std::vector<NODE> *node)
{
  NODE node_tp1, node_tp2, node_tp3; // 记录当前判断网格的三个节点
  // 计算各初始网格单元质量
  for (std::vector<ELEM>::iterator elem_iter = elem->begin(); elem_iter != elem->end(); ++elem_iter)
  {
    // 取出三角形三个顶点NODE类
    node_tp1 = node->at(elem_iter->get_form(0));
    node_tp2 = node->at(elem_iter->get_form(1));
    node_tp3 = node->at(elem_iter->get_form(2));
    elem_iter->tr_quality(abs(Shape_Quality(node_tp1, node_tp2, node_tp3))); // 此处不需要判断节点顺序，取绝对值，避免负值
  }
  return;
}

// 输出当前三角化的网格单元质量信息
void Quality_Information(std::vector<ELEM> *elem, int *elem_num, std::vector<NODE> *node)
{
  int low_quality_elem = 0;  // 定义一个变量，储存劣质网格数量
  double all_quality = 0;    // 定义一个变量，储存所有网格单元的质量，初始化为0
  double lowest_quality = 1; // 定义一个变量，储存网格单元的最小质量，初始化为1
  // 搜索各网格单元质量
  for (std::vector<ELEM>::iterator elem_iter = elem->begin(); elem_iter != elem->end(); ++elem_iter)
  {
    if (elem_iter->get_quality() < Minimum_Shape_Quality_Smoothing)
      low_quality_elem++;
    all_quality += elem_iter->get_quality();
    lowest_quality = elem_iter->get_quality() < lowest_quality ? elem_iter->get_quality() : lowest_quality;
  }
  std::cout << "The average quality of the current triangulation is " << all_quality / *elem_num << ".\n";
  std::cout << "The lowest quality of the current triangulation is " << lowest_quality << ".\n";
  std::cout << "The number of Bad Meshes is " << low_quality_elem << std::endl;
  return;
}

// 利用拓扑变换实现网格质量优化，此处具体实现方式是边交换技术
void Quality_Optimization_EdgeSwapping(std::vector<ELEM> *elem, int *elem_num, std::vector<NODE> *node, int *node_num, int borderNode_num)
{
  int elemNum_opp;              // 声明一个变量，储存待判断网格单元的相邻网格单元编号
  EDGE edge_tp;                 // 声明一个EDGE类，储存待交换边，即两个网格单元对角交换后会删除的边
  NODE currentElem_node;        // 声明一个NODE类，储存当前判断单元中与待交换网格边相对的节点
  NODE oppElem_node;            // 声明一个NODE类，储存当前判断单元相邻网格单元中与待交换网格边相对的节点
  double shape_quality_current; // 声明一个变量，储存边交换前，两个三角形的总质量
  double shape_quality_after;   // 声明一个变量，储存边交换后，两个三角形的总低质量
  // 直接遍历每个网格，判断每个网格的三个相邻网格（最多是三个），当能提高共用一条内边的两个三角形的总质量，
  // 并且其中一个三角形的质量小于Minimum_Shape_Quality_Swap时，进行边交换实现局部重构
  for (int elemNum_iter = 0; elemNum_iter < *elem_num; elemNum_iter++)
  {
    // 每个网格单元最多有三个相邻网格
    for (int i = 0; i < DIM + 1; i++)
    {
      if (elem->at(elemNum_iter).get_neig(i) == -1) // 如果当前判断边没有相邻单元，则进入下一次for循环
        continue;
      elemNum_opp = elem->at(elemNum_iter).get_neig(i); // 得到当前判断单元的相邻网格单元编号
      // 得到当前判断单元与elem_tp_opp单元的相邻共用边，即待交换边
      edge_tp = Node_Opposite_Face(elem->at(elemNum_iter), elem->at(elemNum_iter).get_form(i));
      // 如果待交换边是边界边，则跳过
      if (edge_tp.get_form(0) < borderNode_num && edge_tp.get_form(1) < borderNode_num)
        continue;
      currentElem_node = node->at(elem->at(elemNum_iter).get_form(i)); // 得到当前判断单元中与待交换网格边相对的节点
      // 得到当前判断单元相邻网格单元中与待交换网格边相对的节点
      oppElem_node = node->at(elem->at(elemNum_opp).get_form(Face_Opposite_Node(elem->at(elemNum_opp), edge_tp)));
      shape_quality_current = elem->at(elemNum_iter).get_quality() + elem->at(elemNum_opp).get_quality(); // 得到边交换前，两个三角形的总质量
      // 得到边交换后，两个三角形的总质量
      shape_quality_after = abs(Shape_Quality(node->at(edge_tp.get_form(0)), currentElem_node, oppElem_node)) +
                            abs(Shape_Quality(node->at(edge_tp.get_form(1)), currentElem_node, oppElem_node));
      // 如果待交换边edge_tp交换后，两个三角形的总质量提高，并且其中一个三角形的质量小于Minimum_Shape_Quality_Swap时，进行边交换操作，并更新网格质量信息
      if (shape_quality_after > shape_quality_current && (shape_quality_current < Minimum_Shape_Quality_Swap || shape_quality_after < Minimum_Shape_Quality_Swap))
      {
        // 首先判断该两个网格单元组成的四边形是否是凹四边形，由于凹四边形的对角线必定不相交，所以只需要判断该四边形对角线是否相交，若相交则禁止此次对角交换
        if (Concave_Quadrilateral(elem, elemNum_iter, elemNum_opp, node))
          continue;
        TwoElem_DiagonalSwap(elem, node, elemNum_iter, elemNum_opp);
        elem->at(elemNum_iter).tr_quality(abs(Shape_Quality(node->at(elem->at(elemNum_iter).get_form(0)), node->at(elem->at(elemNum_iter).get_form(1)), node->at(elem->at(elemNum_iter).get_form(2)))));
        elem->at(elemNum_opp).tr_quality(abs(Shape_Quality(node->at(elem->at(elemNum_opp).get_form(0)), node->at(elem->at(elemNum_opp).get_form(1)), node->at(elem->at(elemNum_opp).get_form(2)))));
      }
    }
  }
  return;
}

// 利用节点光顺实现网格质量优化
void Quality_Optimization_Smoothing(std::vector<ELEM> *elem, int *elem_num, std::vector<NODE> *node, int *node_num, int borderNode_num)
{
  std::vector<int> Init_inferior_elem; // 声明一个容器，储存初始劣质单元编号
  // 遍历elem容器，储存初始劣质单元编号
  for (int i = 0; i < *elem_num; i++)
    if (elem->at(i).get_quality() < Minimum_Shape_Quality_Smoothing)
      Init_inferior_elem.push_back(i);
  // 构造局部优化域，用待优化节点编号来代表每个优化域，具体优化域则是当前三角化中包含节点的所有网格单元
  std::vector<int> Laplacian_Smoothing_nodeNum; // 声明一个容器，储存需要进行Laplacian光顺的节点编号
  Construct_Local_Optimization_Domain_All(elem, node, borderNode_num, Init_inferior_elem, &Laplacian_Smoothing_nodeNum);
  //  进行智能Laplacian光顺对每一个节点的位置进行调整
  for (std::vector<int>::iterator iter = Laplacian_Smoothing_nodeNum.begin(); iter != Laplacian_Smoothing_nodeNum.end(); ++iter)
    Laplacian_Smoothing(elem, node, *iter);
  // 遍历Laplacian_Smoothing_nodeNum容器（即优化后的所有节点），判断每个节点的ball内是否全是高质量单元
  // 若依然包含劣质单元，则插入Optimization_based_Smoothing_nodeNum容器，进行Optimization_based光顺
  std::vector<int> Optimization_based_Smoothing_nodeNum; // 声明一个容器，储存需要进行Optimization_based光顺的节点编号
  std::vector<int> elemNum_IncludeNode;                  // 声明一个容器，用来储存包含待判断节点的所有网格单元编号
  for (std::vector<int>::iterator nodeNum_iter = Laplacian_Smoothing_nodeNum.begin(); nodeNum_iter != Laplacian_Smoothing_nodeNum.end(); ++nodeNum_iter)
  {
    std::vector<int>().swap(elemNum_IncludeNode); // 初始化elemNum_IncludeNode，并释放容器空间
    FindBall_fast(elem, node, *nodeNum_iter, &elemNum_IncludeNode);
    // 遍历elemNum_IncludeNode，若包含劣质网格则将当前判断节点插入Optimization_based_Smoothing_nodeNum容器
    for (std::vector<int>::iterator elemNum_iter = elemNum_IncludeNode.begin(); elemNum_iter != elemNum_IncludeNode.end(); ++elemNum_iter)
      if (elem->at(*elemNum_iter).get_quality() < Minimum_Shape_Quality_Smoothing)
      {
        Optimization_based_Smoothing_nodeNum.push_back(*nodeNum_iter);
        break;
      }
  }
  // 进行Optimization_based光顺对每一个节点的位置进行调整
  for (std::vector<int>::iterator iter = Optimization_based_Smoothing_nodeNum.begin(); iter != Optimization_based_Smoothing_nodeNum.end(); ++iter)
    Optimization_based_Smoothing_The_all(elem, node, *iter);
  return;
}

int main()
{
  std::vector<ELEM> elem;                    // 创建ELEM类型容器
  int elem_num = -1;                         // 定义网格单元数，并初始化为-1，-1为未输入网格单元
  std::vector<NODE> node;                    // 创建NODE类型容器
  int node_num = -1;                         // 定义总网格节点数，并初始化为-1，-1为未输入网格节点
  int borderNode_num;                        // 定义总边界点数
  int *borderType = new int[100];            // 定义边界点类型指针，并分配一个储存空间，用来储存边界点类型，具体信息查阅ComplexMesh_gen.cpp文件
  double *density_control = new double[100]; // 定义密度控制信息指针，并分配一个储存空间，用来储存密度控制信息，具体信息查阅ComplexMesh_gen.cpp文件
  std::vector<EDGE> border_edge;             // 创建edge类型容器，用来储存输入图形边界边
  int border_edge_num = 0;                   // 定义边界边数，并初始化为0，0为未生成边界边
  // * 读取输入文件
  ReadInFile(&elem, &elem_num, &node, &node_num, &borderNode_num, borderType, density_control, "in.smesh");
  // 生成边界边
  Gen_BorderEdge(node_num, borderType, &border_edge, &border_edge_num);
  // 初始Delaunay三角化方形边框以4倍于输入图形初始方形边框的面积，对称包含输入图形初始方形边框
  // 定义初始Delaunay三角化方形边框的四个顶角节点在node容器内的位置
  int DeFrame_numPos[] = {-1, -1, -1, -1};
  // 生成初始Delaunay三角化，插入初始Delaunay三角化四边形边框的四个顶角节点，输出该三角化信息文件 Ini_Delaunay.smesh
  Output_IniDeDelaunay(&elem, &elem_num, &node, &node_num, borderType, DeFrame_numPos);
  // * 边界点插入
  InsertBoundaryPoint(&elem, &elem_num, &node, borderNode_num);
  // OutputFile(&elem, &elem_num, &node, &node_num, "Boundary_Delaunay.smesh"); // 输出边界点插入后网格信息文件 Boundary_Delaunay.smesh
  // 搜索初始Delaunay三角化方形边框的四个顶角节点与边界点形成的单元，并打上标识
  Search_DeFrame_Grid(&elem, elem_num, DeFrame_numPos);
  // * 内部点生成和插入
  AutoRefine(&elem, &elem_num, &node, &node_num, density_control);
  // 输出包含初始Delaunay三角化方形边框的网格信息文件 Frame_Delaunay.smesh
  // OutputFile(&elem, &elem_num, &node, &node_num, "Frame_Delaunay.smesh");
  // * 通过对角交换实现约束边界恢复
  Constrained_BoundaryRecovery(&elem, &node, &border_edge, &border_edge_num);
  // InsertSteinerPoints(&elem, &elem_num, &node, &node_num, &border_edge, &border_edge_num); // 插入Steiner点实现保形边界恢复
  // 去掉外部单元
  Removal_ExGrid(&elem, &elem_num, &node, &node_num, DeFrame_numPos);
  // 检查elem类中相邻信息的准确性
  Check_ElemAdjacency_accuracy(&elem, &elem_num, &node, &node_num);
  // 检查node类中elem的准确性
  Check_NodeElem_accuracy(&elem, &elem_num, &node, &node_num);
  // 网格优化前，首先计算各初始网格单元质量
  Calculate_Shape_Quality(&elem, &node);
  // 输出网格质量优化前信息文件 Before_Quality.smesh
  OutputFile(&elem, &elem_num, &node, &node_num, "Before_Quality.smesh");
  // 输出优化前三角化的网格单元质量信息
  std::cout << "Output information about the quality of mesh elements triangulated before optimization:\n";
  Quality_Information(&elem, &elem_num, &node);
  for (int i = 0; i < 3; i++)
  {
    //  * 网格质量优化
    // 先利用拓扑变换实现网格质量优化
    Quality_Optimization_EdgeSwapping(&elem, &elem_num, &node, &node_num, borderNode_num);
    // std::string str = std::to_string(i) + "_swap.smesh";
    // OutputFile(&elem, &elem_num, &node, &node_num, str);
    // 后利用节点光顺实现网格质量优化
    Quality_Optimization_Smoothing(&elem, &elem_num, &node, &node_num, borderNode_num);
    // str = std::to_string(i) + "_smoothing.smesh";
    // OutputFile(&elem, &elem_num, &node, &node_num, str);
    //  输出优化后三角化的网格单元质量信息
    std::cout << "Output optimized triangulated mesh element quality information:\n";
    Quality_Information(&elem, &elem_num, &node);
    // OutputFile(&elem, &elem_num, &node, &node_num, str);
  }
  // 输出网格质量优化后信息文件 Final_Delaunay.smesh
  OutputFile(&elem, &elem_num, &node, &node_num, "Final_Delaunay.smesh");
  delete[] borderType;
  delete[] density_control;
  return 0;
}

/**
 * ctrl + k 然后 ctrl + 0 将大括号收缩
 * * std::vector<int>().swap(elemNum_IncludeNode);          // 初始化elemNum_IncludeNode，并释放容器空间
 * * for (std::vector<ELEM>::iterator elem_iter = elem->begin(); elem_iter != elem->end();++elem_iter)
 * * if (std::find(elemNum_wait.begin(), elemNum_wait.end(), elem->at(elemNum_tp).get_neig(i)) == elemNum_wait.end())
 * * for (std::vector<EDGE>::iterator edge_iter = border_edge->begin(); edge_iter != border_edge->end(); ++edge_iter)
 */

/**
 * * 以下所有代码都是采用的Shewchuk基于自适应精度算术实现的几何谓词incircle
 */

#define INEXACT /* Nothing */
/* #define INEXACT volatile */

#define REALPRINT doubleprint
#define REALRAND doublerand
#define NARROWRAND narrowdoublerand
#define UNIFORMRAND uniformdoublerand

/*
以下两种查找绝对值的方法中哪一种最快取决于编译器。
一些编译器可以内联和优化 fabs()调用；但大多数会招致函数调用的开销，这是灾难性的缓慢。
在IEEE机器上更快的方法可能是屏蔽适当的位，但这在C中很难做到。
*/
#define Absolute(a) ((a) >= 0.0 ? (a) : -(a))
/* #define Absolute(a)  fabs(a) */

/*
许多操作分为两部分，执行近似操作的主要部分和计算该操作的舍入误差的“尾部”。
Fast_Two_Sum()、Fast_Two_Diff()、Two_Sum()、Two_Diff()、Split()和Two_Product()操作均按照参考中的描述实现。
这些宏中的每一个都需要在调用例程中定义某些变量。
变量“bvirt”、“c”、“abig”、“_i”、“_j”、“_k”、“_l”、“_m”和“_n”被声明为“不精确”，因为它们存储了一个可能导致舍入误差的操作。
输入参数“x”（或编号最大的“x_”参数）也必须声明为“INEXACT”。
*/

#define Fast_Two_Sum_Tail(a, b, x, y) \
  bvirt = x - a;                      \
  y = b - bvirt

#define Fast_Two_Sum(a, b, x, y) \
  x = (REAL)(a + b);             \
  Fast_Two_Sum_Tail(a, b, x, y)

#define Fast_Two_Diff_Tail(a, b, x, y) \
  bvirt = a - x;                       \
  y = bvirt - b

#define Fast_Two_Diff(a, b, x, y) \
  x = (REAL)(a - b);              \
  Fast_Two_Diff_Tail(a, b, x, y)

#define Two_Sum_Tail(a, b, x, y) \
  bvirt = (REAL)(x - a);         \
  avirt = x - bvirt;             \
  bround = b - bvirt;            \
  around = a - avirt;            \
  y = around + bround

#define Two_Sum(a, b, x, y) \
  x = (REAL)(a + b);        \
  Two_Sum_Tail(a, b, x, y)

#define Two_Diff_Tail(a, b, x, y) \
  bvirt = (REAL)(a - x);          \
  avirt = x + bvirt;              \
  bround = bvirt - b;             \
  around = a - avirt;             \
  y = around + bround

#define Two_Diff(a, b, x, y) \
  x = (REAL)(a - b);         \
  Two_Diff_Tail(a, b, x, y)

#define Split(a, ahi, alo)  \
  c = (REAL)(splitter * a); \
  abig = (REAL)(c - a);     \
  ahi = c - abig;           \
  alo = a - ahi

#define Two_Product_Tail(a, b, x, y) \
  Split(a, ahi, alo);                \
  Split(b, bhi, blo);                \
  err1 = x - (ahi * bhi);            \
  err2 = err1 - (alo * bhi);         \
  err3 = err2 - (ahi * blo);         \
  y = (alo * blo) - err3

#define Two_Product(a, b, x, y) \
  x = (REAL)(a * b);            \
  Two_Product_Tail(a, b, x, y)

// Two_Product_Presplit()是 Two_Product()其中一个输入已经被拆分。避免冗余拆分。
/* Two_Product_Presplit() is Two_Product() where one of the inputs has       */
/*   already been split.  Avoids redundant splitting.                        */
#define Two_Product_Presplit(a, b, bhi, blo, x, y) \
  x = (REAL)(a * b);                               \
  Split(a, ahi, alo);                              \
  err1 = x - (ahi * bhi);                          \
  err2 = err1 - (alo * bhi);                       \
  err3 = err2 - (ahi * blo);                       \
  y = (alo * blo) - err3

// Square() can be done more quickly than Two_Product().
#define Square_Tail(a, x, y)         \
  Split(a, ahi, alo);                \
  err1 = x - (ahi * ahi);            \
  err3 = err1 - ((ahi + ahi) * alo); \
  y = (alo * alo) - err3

#define Square(a, x, y) \
  x = (REAL)(a * a);    \
  Square_Tail(a, x, y)

// 用于求和各种固定长度扩展的宏。这些都是Expansion_Sum()的展开版本。
#define Two_One_Sum(a1, a0, b, x2, x1, x0) \
  Two_Sum(a0, b, _i, x0);                  \
  Two_Sum(a1, _i, x2, x1)

#define Two_One_Diff(a1, a0, b, x2, x1, x0) \
  Two_Diff(a0, b, _i, x0);                  \
  Two_Sum(a1, _i, x2, x1)

#define Two_Two_Sum(a1, a0, b1, b0, x3, x2, x1, x0) \
  Two_One_Sum(a1, a0, b0, _j, _0, x0);              \
  Two_One_Sum(_j, _0, b1, x3, x2, x1)

#define Two_Two_Diff(a1, a0, b1, b0, x3, x2, x1, x0) \
  Two_One_Diff(a1, a0, b0, _j, _0, x0);              \
  Two_One_Diff(_j, _0, b1, x3, x2, x1)

REAL splitter; /* = 2^ceiling(p / 2) + 1.  用于将浮点数分成两半。 */
REAL epsilon;  /* = 2^(-p).  用于估计舍入误差。 */
/* 一组用于计算最大舍入误差的系数。          */
REAL resulterrbound;
REAL ccwerrboundA, ccwerrboundB, ccwerrboundC;
REAL o3derrboundA, o3derrboundB, o3derrboundC;
REAL iccerrboundA, iccerrboundB, iccerrboundC;
REAL isperrboundA, isperrboundB, isperrboundC;

// estimate() 产生一个扩展值的单字估计。
REAL estimate(int elen, REAL *e)
{
  REAL Q;
  int eindex;

  Q = e[0];
  for (eindex = 1; eindex < elen; eindex++)
  {
    Q += e[eindex];
  }
  return Q;
}

/*
scale_expansion_zeroelim()将展开乘以标量，从输出展开中消除零分量。设置h=be。
有关详细信息，请参阅Shewchuk的论文的任一版本。
保持非重叠属性。如果使用舍入到偶数（与IEEE 754一样），则也保持强不重叠和不相邻的属性。
（也就是说，如果e具有这些属性之一，那么h也具有。）
*/

int scale_expansion_zeroelim(int elen, REAL *e, REAL b, REAL *h) /* e and h cannot be the same. */
{
  INEXACT REAL Q, sum;
  REAL hh;
  INEXACT REAL product1;
  REAL product0;
  int eindex, hindex;
  REAL enow;
  INEXACT REAL bvirt;
  REAL avirt, bround, around;
  INEXACT REAL c;
  INEXACT REAL abig;
  REAL ahi, alo, bhi, blo;
  REAL err1, err2, err3;

  Split(b, bhi, blo);
  Two_Product_Presplit(e[0], b, bhi, blo, Q, hh);
  hindex = 0;
  if (hh != 0)
  {
    h[hindex++] = hh;
  }
  for (eindex = 1; eindex < elen; eindex++)
  {
    enow = e[eindex];
    Two_Product_Presplit(enow, b, bhi, blo, product1, product0);
    Two_Sum(Q, product0, sum, hh);
    if (hh != 0)
    {
      h[hindex++] = hh;
    }
    Fast_Two_Sum(product1, sum, Q, hh);
    if (hh != 0)
    {
      h[hindex++] = hh;
    }
  }
  if ((Q != 0.0) || (hindex == 0))
  {
    h[hindex++] = Q;
  }
  return hindex;
}

/*
fast_expansion_sum_zeroelim()对两个扩展求和，从输出扩展中消除零分量。设置h=e+f。
有关详细信息，请参阅Shewchuk的论文的长版。
如果使用舍入到偶数（与IEEE 754一样），则保持强不重叠属性。
（也就是说，如果e是强不重叠的，h也将是。）不保持不重叠或不相邻的属性。
*/

int fast_expansion_sum_zeroelim(int elen, REAL *e, int flen, REAL *f, REAL *h) /* h cannot be e or f. */
{
  REAL Q;
  INEXACT REAL Qnew;
  INEXACT REAL hh;
  INEXACT REAL bvirt;
  REAL avirt, bround, around;
  int eindex, findex, hindex;
  REAL enow, fnow;

  enow = e[0];
  fnow = f[0];
  eindex = findex = 0;
  if ((fnow > enow) == (fnow > -enow))
  {
    Q = enow;
    enow = e[++eindex];
  }
  else
  {
    Q = fnow;
    fnow = f[++findex];
  }
  hindex = 0;
  if ((eindex < elen) && (findex < flen))
  {
    if ((fnow > enow) == (fnow > -enow))
    {
      Fast_Two_Sum(enow, Q, Qnew, hh);
      enow = e[++eindex];
    }
    else
    {
      Fast_Two_Sum(fnow, Q, Qnew, hh);
      fnow = f[++findex];
    }
    Q = Qnew;
    if (hh != 0.0)
    {
      h[hindex++] = hh;
    }
    while ((eindex < elen) && (findex < flen))
    {
      if ((fnow > enow) == (fnow > -enow))
      {
        Two_Sum(Q, enow, Qnew, hh);
        enow = e[++eindex];
      }
      else
      {
        Two_Sum(Q, fnow, Qnew, hh);
        fnow = f[++findex];
      }
      Q = Qnew;
      if (hh != 0.0)
      {
        h[hindex++] = hh;
      }
    }
  }
  while (eindex < elen)
  {
    Two_Sum(Q, enow, Qnew, hh);
    enow = e[++eindex];
    Q = Qnew;
    if (hh != 0.0)
    {
      h[hindex++] = hh;
    }
  }
  while (findex < flen)
  {
    Two_Sum(Q, fnow, Qnew, hh);
    fnow = f[++findex];
    Q = Qnew;
    if (hh != 0.0)
    {
      h[hindex++] = hh;
    }
  }
  if ((Q != 0.0) || (hindex == 0))
  {
    h[hindex++] = Q;
  }
  return hindex;
}

/*
orient2d()自适应精确二维方向测试。强壮的。
如果点pa、pb、pc按逆时针顺序出现则返回正值；如果它们按顺时针顺序出现，则为负值；
如果它们共线则为零。结果也是由三个点定义的三角形的符号面积的两倍的粗略近似值。
使用精确算术来确保正确答案。返回的结果是矩阵的行列式。
仅在orient2d()中，此行列式是自适应计算的，从某种意义上说，精确算术仅用于确保返回值具有正确符号所需的程度。
因此，orient2d()通常非常快，但当输入点共线或接近共线时运行速度会更慢。
*/

REAL orient2dadapt(REAL *pa, REAL *pb, REAL *pc, REAL detsum)
{
  INEXACT REAL acx, acy, bcx, bcy;
  REAL acxtail, acytail, bcxtail, bcytail;
  INEXACT REAL detleft, detright;
  REAL detlefttail, detrighttail;
  REAL det, errbound;
  REAL B[4], C1[8], C2[12], D[16];
  INEXACT REAL B3;
  int C1length, C2length, Dlength;
  REAL u[4];
  INEXACT REAL u3;
  INEXACT REAL s1, t1;
  REAL s0, t0;

  INEXACT REAL bvirt;
  REAL avirt, bround, around;
  INEXACT REAL c;
  INEXACT REAL abig;
  REAL ahi, alo, bhi, blo;
  REAL err1, err2, err3;
  INEXACT REAL _i, _j;
  REAL _0;

  acx = (REAL)(pa[0] - pc[0]);
  bcx = (REAL)(pb[0] - pc[0]);
  acy = (REAL)(pa[1] - pc[1]);
  bcy = (REAL)(pb[1] - pc[1]);

  Two_Product(acx, bcy, detleft, detlefttail);
  Two_Product(acy, bcx, detright, detrighttail);

  Two_Two_Diff(detleft, detlefttail, detright, detrighttail,
               B3, B[2], B[1], B[0]);
  B[3] = B3;

  det = estimate(4, B);
  errbound = ccwerrboundB * detsum;
  if ((det >= errbound) || (-det >= errbound))
  {
    return det;
  }

  Two_Diff_Tail(pa[0], pc[0], acx, acxtail);
  Two_Diff_Tail(pb[0], pc[0], bcx, bcxtail);
  Two_Diff_Tail(pa[1], pc[1], acy, acytail);
  Two_Diff_Tail(pb[1], pc[1], bcy, bcytail);

  if ((acxtail == 0.0) && (acytail == 0.0) && (bcxtail == 0.0) && (bcytail == 0.0))
  {
    return det;
  }

  errbound = ccwerrboundC * detsum + resulterrbound * Absolute(det);
  det += (acx * bcytail + bcy * acxtail) - (acy * bcxtail + bcx * acytail);
  if ((det >= errbound) || (-det >= errbound))
  {
    return det;
  }

  Two_Product(acxtail, bcy, s1, s0);
  Two_Product(acytail, bcx, t1, t0);
  Two_Two_Diff(s1, s0, t1, t0, u3, u[2], u[1], u[0]);
  u[3] = u3;
  C1length = fast_expansion_sum_zeroelim(4, B, 4, u, C1);

  Two_Product(acx, bcytail, s1, s0);
  Two_Product(acy, bcxtail, t1, t0);
  Two_Two_Diff(s1, s0, t1, t0, u3, u[2], u[1], u[0]);
  u[3] = u3;
  C2length = fast_expansion_sum_zeroelim(C1length, C1, 4, u, C2);

  Two_Product(acxtail, bcytail, s1, s0);
  Two_Product(acytail, bcxtail, t1, t0);
  Two_Two_Diff(s1, s0, t1, t0, u3, u[2], u[1], u[0]);
  u[3] = u3;
  Dlength = fast_expansion_sum_zeroelim(C2length, C2, 4, u, D);

  return (D[Dlength - 1]);
}

REAL orient2d(REAL *pa, REAL *pb, REAL *pc)
{
  REAL detleft, detright, det;
  REAL detsum, errbound;

  detleft = (pa[0] - pc[0]) * (pb[1] - pc[1]);
  detright = (pa[1] - pc[1]) * (pb[0] - pc[0]);
  det = detleft - detright;

  if (detleft > 0.0)
  {
    if (detright <= 0.0)
    {
      return det;
    }
    else
    {
      detsum = detleft + detright;
    }
  }
  else if (detleft < 0.0)
  {
    if (detright >= 0.0)
    {
      return det;
    }
    else
    {
      detsum = -detleft - detright;
    }
  }
  else
  {
    return det;
  }

  errbound = ccwerrboundA * detsum;
  if ((det >= errbound) || (-det >= errbound))
  {
    return det;
  }

  return orient2dadapt(pa, pb, pc, detsum);
}

/*
incircle()自适应精确二维内切圆测试。强壮的。
如果点pd位于通过pa、pb 和 pc的圆内，则返回正值；如果它在外面，则为负值；如果四个点共圆则为零。
pa、pb、pc点必须按逆时针顺序排列，否则结果的符号会颠倒。
使用精确算术来确保正确答案。返回的结果是矩阵的行列式。
仅在incircle()中，此行列式是自适应计算的，从某种意义上说，精确算术仅用于确保返回值具有正确符号所需的程度。
因此，incircle()通常非常快，但当输入点同圆或接近同圆时运行速度会更慢。
*/

REAL incircleadapt(REAL *pa, REAL *pb, REAL *pc, REAL *pd, REAL permanent)
{
  INEXACT REAL adx, bdx, cdx, ady, bdy, cdy;
  REAL det, errbound;

  INEXACT REAL bdxcdy1, cdxbdy1, cdxady1, adxcdy1, adxbdy1, bdxady1;
  REAL bdxcdy0, cdxbdy0, cdxady0, adxcdy0, adxbdy0, bdxady0;
  REAL bc[4], ca[4], ab[4];
  INEXACT REAL bc3, ca3, ab3;
  REAL axbc[8], axxbc[16], aybc[8], ayybc[16], adet[32];
  int axbclen, axxbclen, aybclen, ayybclen, alen;
  REAL bxca[8], bxxca[16], byca[8], byyca[16], bdet[32];
  int bxcalen, bxxcalen, bycalen, byycalen, blen;
  REAL cxab[8], cxxab[16], cyab[8], cyyab[16], cdet[32];
  int cxablen, cxxablen, cyablen, cyyablen, clen;
  REAL abdet[64];
  int ablen;
  REAL fin1[1152], fin2[1152];
  REAL *finnow, *finother, *finswap;
  int finlength;

  REAL adxtail, bdxtail, cdxtail, adytail, bdytail, cdytail;
  INEXACT REAL adxadx1, adyady1, bdxbdx1, bdybdy1, cdxcdx1, cdycdy1;
  REAL adxadx0, adyady0, bdxbdx0, bdybdy0, cdxcdx0, cdycdy0;
  REAL aa[4], bb[4], cc[4];
  INEXACT REAL aa3, bb3, cc3;
  INEXACT REAL ti1, tj1;
  REAL ti0, tj0;
  REAL u[4], v[4];
  INEXACT REAL u3, v3;
  REAL temp8[8], temp16a[16], temp16b[16], temp16c[16];
  REAL temp32a[32], temp32b[32], temp48[48], temp64[64];
  int temp8len, temp16alen, temp16blen, temp16clen;
  int temp32alen, temp32blen, temp48len, temp64len;
  REAL axtbb[8], axtcc[8], aytbb[8], aytcc[8];
  int axtbblen, axtcclen, aytbblen, aytcclen;
  REAL bxtaa[8], bxtcc[8], bytaa[8], bytcc[8];
  int bxtaalen, bxtcclen, bytaalen, bytcclen;
  REAL cxtaa[8], cxtbb[8], cytaa[8], cytbb[8];
  int cxtaalen, cxtbblen, cytaalen, cytbblen;
  REAL axtbc[8], aytbc[8], bxtca[8], bytca[8], cxtab[8], cytab[8];
  int axtbclen, aytbclen, bxtcalen, bytcalen, cxtablen, cytablen;
  REAL axtbct[16], aytbct[16], bxtcat[16], bytcat[16], cxtabt[16], cytabt[16];
  int axtbctlen, aytbctlen, bxtcatlen, bytcatlen, cxtabtlen, cytabtlen;
  REAL axtbctt[8], aytbctt[8], bxtcatt[8];
  REAL bytcatt[8], cxtabtt[8], cytabtt[8];
  int axtbcttlen, aytbcttlen, bxtcattlen, bytcattlen, cxtabttlen, cytabttlen;
  REAL abt[8], bct[8], cat[8];
  int abtlen, bctlen, catlen;
  REAL abtt[4], bctt[4], catt[4];
  int abttlen, bcttlen, cattlen;
  INEXACT REAL abtt3, bctt3, catt3;
  REAL negate;

  INEXACT REAL bvirt;
  REAL avirt, bround, around;
  INEXACT REAL c;
  INEXACT REAL abig;
  REAL ahi, alo, bhi, blo;
  REAL err1, err2, err3;
  INEXACT REAL _i, _j;
  REAL _0;

  adx = (REAL)(pa[0] - pd[0]);
  bdx = (REAL)(pb[0] - pd[0]);
  cdx = (REAL)(pc[0] - pd[0]);
  ady = (REAL)(pa[1] - pd[1]);
  bdy = (REAL)(pb[1] - pd[1]);
  cdy = (REAL)(pc[1] - pd[1]);

  Two_Product(bdx, cdy, bdxcdy1, bdxcdy0);
  Two_Product(cdx, bdy, cdxbdy1, cdxbdy0);
  Two_Two_Diff(bdxcdy1, bdxcdy0, cdxbdy1, cdxbdy0, bc3, bc[2], bc[1], bc[0]);
  bc[3] = bc3;
  axbclen = scale_expansion_zeroelim(4, bc, adx, axbc);
  axxbclen = scale_expansion_zeroelim(axbclen, axbc, adx, axxbc);
  aybclen = scale_expansion_zeroelim(4, bc, ady, aybc);
  ayybclen = scale_expansion_zeroelim(aybclen, aybc, ady, ayybc);
  alen = fast_expansion_sum_zeroelim(axxbclen, axxbc, ayybclen, ayybc, adet);

  Two_Product(cdx, ady, cdxady1, cdxady0);
  Two_Product(adx, cdy, adxcdy1, adxcdy0);
  Two_Two_Diff(cdxady1, cdxady0, adxcdy1, adxcdy0, ca3, ca[2], ca[1], ca[0]);
  ca[3] = ca3;
  bxcalen = scale_expansion_zeroelim(4, ca, bdx, bxca);
  bxxcalen = scale_expansion_zeroelim(bxcalen, bxca, bdx, bxxca);
  bycalen = scale_expansion_zeroelim(4, ca, bdy, byca);
  byycalen = scale_expansion_zeroelim(bycalen, byca, bdy, byyca);
  blen = fast_expansion_sum_zeroelim(bxxcalen, bxxca, byycalen, byyca, bdet);

  Two_Product(adx, bdy, adxbdy1, adxbdy0);
  Two_Product(bdx, ady, bdxady1, bdxady0);
  Two_Two_Diff(adxbdy1, adxbdy0, bdxady1, bdxady0, ab3, ab[2], ab[1], ab[0]);
  ab[3] = ab3;
  cxablen = scale_expansion_zeroelim(4, ab, cdx, cxab);
  cxxablen = scale_expansion_zeroelim(cxablen, cxab, cdx, cxxab);
  cyablen = scale_expansion_zeroelim(4, ab, cdy, cyab);
  cyyablen = scale_expansion_zeroelim(cyablen, cyab, cdy, cyyab);
  clen = fast_expansion_sum_zeroelim(cxxablen, cxxab, cyyablen, cyyab, cdet);

  ablen = fast_expansion_sum_zeroelim(alen, adet, blen, bdet, abdet);
  finlength = fast_expansion_sum_zeroelim(ablen, abdet, clen, cdet, fin1);

  det = estimate(finlength, fin1);
  errbound = iccerrboundB * permanent;
  if ((det >= errbound) || (-det >= errbound))
  {
    return det;
  }

  Two_Diff_Tail(pa[0], pd[0], adx, adxtail);
  Two_Diff_Tail(pa[1], pd[1], ady, adytail);
  Two_Diff_Tail(pb[0], pd[0], bdx, bdxtail);
  Two_Diff_Tail(pb[1], pd[1], bdy, bdytail);
  Two_Diff_Tail(pc[0], pd[0], cdx, cdxtail);
  Two_Diff_Tail(pc[1], pd[1], cdy, cdytail);
  if ((adxtail == 0.0) && (bdxtail == 0.0) && (cdxtail == 0.0) && (adytail == 0.0) && (bdytail == 0.0) && (cdytail == 0.0))
  {
    return det;
  }

  errbound = iccerrboundC * permanent + resulterrbound * Absolute(det);
  det += ((adx * adx + ady * ady) * ((bdx * cdytail + cdy * bdxtail) - (bdy * cdxtail + cdx * bdytail)) + 2.0 * (adx * adxtail + ady * adytail) * (bdx * cdy - bdy * cdx)) + ((bdx * bdx + bdy * bdy) * ((cdx * adytail + ady * cdxtail) - (cdy * adxtail + adx * cdytail)) + 2.0 * (bdx * bdxtail + bdy * bdytail) * (cdx * ady - cdy * adx)) + ((cdx * cdx + cdy * cdy) * ((adx * bdytail + bdy * adxtail) - (ady * bdxtail + bdx * adytail)) + 2.0 * (cdx * cdxtail + cdy * cdytail) * (adx * bdy - ady * bdx));
  if ((det >= errbound) || (-det >= errbound))
  {
    return det;
  }

  finnow = fin1;
  finother = fin2;

  if ((bdxtail != 0.0) || (bdytail != 0.0) || (cdxtail != 0.0) || (cdytail != 0.0))
  {
    Square(adx, adxadx1, adxadx0);
    Square(ady, adyady1, adyady0);
    Two_Two_Sum(adxadx1, adxadx0, adyady1, adyady0, aa3, aa[2], aa[1], aa[0]);
    aa[3] = aa3;
  }
  if ((cdxtail != 0.0) || (cdytail != 0.0) || (adxtail != 0.0) || (adytail != 0.0))
  {
    Square(bdx, bdxbdx1, bdxbdx0);
    Square(bdy, bdybdy1, bdybdy0);
    Two_Two_Sum(bdxbdx1, bdxbdx0, bdybdy1, bdybdy0, bb3, bb[2], bb[1], bb[0]);
    bb[3] = bb3;
  }
  if ((adxtail != 0.0) || (adytail != 0.0) || (bdxtail != 0.0) || (bdytail != 0.0))
  {
    Square(cdx, cdxcdx1, cdxcdx0);
    Square(cdy, cdycdy1, cdycdy0);
    Two_Two_Sum(cdxcdx1, cdxcdx0, cdycdy1, cdycdy0, cc3, cc[2], cc[1], cc[0]);
    cc[3] = cc3;
  }

  if (adxtail != 0.0)
  {
    axtbclen = scale_expansion_zeroelim(4, bc, adxtail, axtbc);
    temp16alen = scale_expansion_zeroelim(axtbclen, axtbc, 2.0 * adx,
                                          temp16a);

    axtcclen = scale_expansion_zeroelim(4, cc, adxtail, axtcc);
    temp16blen = scale_expansion_zeroelim(axtcclen, axtcc, bdy, temp16b);

    axtbblen = scale_expansion_zeroelim(4, bb, adxtail, axtbb);
    temp16clen = scale_expansion_zeroelim(axtbblen, axtbb, -cdy, temp16c);

    temp32alen = fast_expansion_sum_zeroelim(temp16alen, temp16a,
                                             temp16blen, temp16b, temp32a);
    temp48len = fast_expansion_sum_zeroelim(temp16clen, temp16c,
                                            temp32alen, temp32a, temp48);
    finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp48len,
                                            temp48, finother);
    finswap = finnow;
    finnow = finother;
    finother = finswap;
  }
  if (adytail != 0.0)
  {
    aytbclen = scale_expansion_zeroelim(4, bc, adytail, aytbc);
    temp16alen = scale_expansion_zeroelim(aytbclen, aytbc, 2.0 * ady,
                                          temp16a);

    aytbblen = scale_expansion_zeroelim(4, bb, adytail, aytbb);
    temp16blen = scale_expansion_zeroelim(aytbblen, aytbb, cdx, temp16b);

    aytcclen = scale_expansion_zeroelim(4, cc, adytail, aytcc);
    temp16clen = scale_expansion_zeroelim(aytcclen, aytcc, -bdx, temp16c);

    temp32alen = fast_expansion_sum_zeroelim(temp16alen, temp16a,
                                             temp16blen, temp16b, temp32a);
    temp48len = fast_expansion_sum_zeroelim(temp16clen, temp16c,
                                            temp32alen, temp32a, temp48);
    finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp48len,
                                            temp48, finother);
    finswap = finnow;
    finnow = finother;
    finother = finswap;
  }
  if (bdxtail != 0.0)
  {
    bxtcalen = scale_expansion_zeroelim(4, ca, bdxtail, bxtca);
    temp16alen = scale_expansion_zeroelim(bxtcalen, bxtca, 2.0 * bdx,
                                          temp16a);

    bxtaalen = scale_expansion_zeroelim(4, aa, bdxtail, bxtaa);
    temp16blen = scale_expansion_zeroelim(bxtaalen, bxtaa, cdy, temp16b);

    bxtcclen = scale_expansion_zeroelim(4, cc, bdxtail, bxtcc);
    temp16clen = scale_expansion_zeroelim(bxtcclen, bxtcc, -ady, temp16c);

    temp32alen = fast_expansion_sum_zeroelim(temp16alen, temp16a,
                                             temp16blen, temp16b, temp32a);
    temp48len = fast_expansion_sum_zeroelim(temp16clen, temp16c,
                                            temp32alen, temp32a, temp48);
    finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp48len,
                                            temp48, finother);
    finswap = finnow;
    finnow = finother;
    finother = finswap;
  }
  if (bdytail != 0.0)
  {
    bytcalen = scale_expansion_zeroelim(4, ca, bdytail, bytca);
    temp16alen = scale_expansion_zeroelim(bytcalen, bytca, 2.0 * bdy,
                                          temp16a);

    bytcclen = scale_expansion_zeroelim(4, cc, bdytail, bytcc);
    temp16blen = scale_expansion_zeroelim(bytcclen, bytcc, adx, temp16b);

    bytaalen = scale_expansion_zeroelim(4, aa, bdytail, bytaa);
    temp16clen = scale_expansion_zeroelim(bytaalen, bytaa, -cdx, temp16c);

    temp32alen = fast_expansion_sum_zeroelim(temp16alen, temp16a,
                                             temp16blen, temp16b, temp32a);
    temp48len = fast_expansion_sum_zeroelim(temp16clen, temp16c,
                                            temp32alen, temp32a, temp48);
    finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp48len,
                                            temp48, finother);
    finswap = finnow;
    finnow = finother;
    finother = finswap;
  }
  if (cdxtail != 0.0)
  {
    cxtablen = scale_expansion_zeroelim(4, ab, cdxtail, cxtab);
    temp16alen = scale_expansion_zeroelim(cxtablen, cxtab, 2.0 * cdx,
                                          temp16a);

    cxtbblen = scale_expansion_zeroelim(4, bb, cdxtail, cxtbb);
    temp16blen = scale_expansion_zeroelim(cxtbblen, cxtbb, ady, temp16b);

    cxtaalen = scale_expansion_zeroelim(4, aa, cdxtail, cxtaa);
    temp16clen = scale_expansion_zeroelim(cxtaalen, cxtaa, -bdy, temp16c);

    temp32alen = fast_expansion_sum_zeroelim(temp16alen, temp16a,
                                             temp16blen, temp16b, temp32a);
    temp48len = fast_expansion_sum_zeroelim(temp16clen, temp16c,
                                            temp32alen, temp32a, temp48);
    finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp48len,
                                            temp48, finother);
    finswap = finnow;
    finnow = finother;
    finother = finswap;
  }
  if (cdytail != 0.0)
  {
    cytablen = scale_expansion_zeroelim(4, ab, cdytail, cytab);
    temp16alen = scale_expansion_zeroelim(cytablen, cytab, 2.0 * cdy,
                                          temp16a);

    cytaalen = scale_expansion_zeroelim(4, aa, cdytail, cytaa);
    temp16blen = scale_expansion_zeroelim(cytaalen, cytaa, bdx, temp16b);

    cytbblen = scale_expansion_zeroelim(4, bb, cdytail, cytbb);
    temp16clen = scale_expansion_zeroelim(cytbblen, cytbb, -adx, temp16c);

    temp32alen = fast_expansion_sum_zeroelim(temp16alen, temp16a,
                                             temp16blen, temp16b, temp32a);
    temp48len = fast_expansion_sum_zeroelim(temp16clen, temp16c,
                                            temp32alen, temp32a, temp48);
    finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp48len,
                                            temp48, finother);
    finswap = finnow;
    finnow = finother;
    finother = finswap;
  }

  if ((adxtail != 0.0) || (adytail != 0.0))
  {
    if ((bdxtail != 0.0) || (bdytail != 0.0) || (cdxtail != 0.0) || (cdytail != 0.0))
    {
      Two_Product(bdxtail, cdy, ti1, ti0);
      Two_Product(bdx, cdytail, tj1, tj0);
      Two_Two_Sum(ti1, ti0, tj1, tj0, u3, u[2], u[1], u[0]);
      u[3] = u3;
      negate = -bdy;
      Two_Product(cdxtail, negate, ti1, ti0);
      negate = -bdytail;
      Two_Product(cdx, negate, tj1, tj0);
      Two_Two_Sum(ti1, ti0, tj1, tj0, v3, v[2], v[1], v[0]);
      v[3] = v3;
      bctlen = fast_expansion_sum_zeroelim(4, u, 4, v, bct);

      Two_Product(bdxtail, cdytail, ti1, ti0);
      Two_Product(cdxtail, bdytail, tj1, tj0);
      Two_Two_Diff(ti1, ti0, tj1, tj0, bctt3, bctt[2], bctt[1], bctt[0]);
      bctt[3] = bctt3;
      bcttlen = 4;
    }
    else
    {
      bct[0] = 0.0;
      bctlen = 1;
      bctt[0] = 0.0;
      bcttlen = 1;
    }

    if (adxtail != 0.0)
    {
      temp16alen = scale_expansion_zeroelim(axtbclen, axtbc, adxtail, temp16a);
      axtbctlen = scale_expansion_zeroelim(bctlen, bct, adxtail, axtbct);
      temp32alen = scale_expansion_zeroelim(axtbctlen, axtbct, 2.0 * adx,
                                            temp32a);
      temp48len = fast_expansion_sum_zeroelim(temp16alen, temp16a,
                                              temp32alen, temp32a, temp48);
      finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp48len,
                                              temp48, finother);
      finswap = finnow;
      finnow = finother;
      finother = finswap;
      if (bdytail != 0.0)
      {
        temp8len = scale_expansion_zeroelim(4, cc, adxtail, temp8);
        temp16alen = scale_expansion_zeroelim(temp8len, temp8, bdytail,
                                              temp16a);
        finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp16alen,
                                                temp16a, finother);
        finswap = finnow;
        finnow = finother;
        finother = finswap;
      }
      if (cdytail != 0.0)
      {
        temp8len = scale_expansion_zeroelim(4, bb, -adxtail, temp8);
        temp16alen = scale_expansion_zeroelim(temp8len, temp8, cdytail,
                                              temp16a);
        finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp16alen,
                                                temp16a, finother);
        finswap = finnow;
        finnow = finother;
        finother = finswap;
      }

      temp32alen = scale_expansion_zeroelim(axtbctlen, axtbct, adxtail,
                                            temp32a);
      axtbcttlen = scale_expansion_zeroelim(bcttlen, bctt, adxtail, axtbctt);
      temp16alen = scale_expansion_zeroelim(axtbcttlen, axtbctt, 2.0 * adx,
                                            temp16a);
      temp16blen = scale_expansion_zeroelim(axtbcttlen, axtbctt, adxtail,
                                            temp16b);
      temp32blen = fast_expansion_sum_zeroelim(temp16alen, temp16a,
                                               temp16blen, temp16b, temp32b);
      temp64len = fast_expansion_sum_zeroelim(temp32alen, temp32a,
                                              temp32blen, temp32b, temp64);
      finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp64len,
                                              temp64, finother);
      finswap = finnow;
      finnow = finother;
      finother = finswap;
    }
    if (adytail != 0.0)
    {
      temp16alen = scale_expansion_zeroelim(aytbclen, aytbc, adytail, temp16a);
      aytbctlen = scale_expansion_zeroelim(bctlen, bct, adytail, aytbct);
      temp32alen = scale_expansion_zeroelim(aytbctlen, aytbct, 2.0 * ady,
                                            temp32a);
      temp48len = fast_expansion_sum_zeroelim(temp16alen, temp16a,
                                              temp32alen, temp32a, temp48);
      finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp48len,
                                              temp48, finother);
      finswap = finnow;
      finnow = finother;
      finother = finswap;

      temp32alen = scale_expansion_zeroelim(aytbctlen, aytbct, adytail,
                                            temp32a);
      aytbcttlen = scale_expansion_zeroelim(bcttlen, bctt, adytail, aytbctt);
      temp16alen = scale_expansion_zeroelim(aytbcttlen, aytbctt, 2.0 * ady,
                                            temp16a);
      temp16blen = scale_expansion_zeroelim(aytbcttlen, aytbctt, adytail,
                                            temp16b);
      temp32blen = fast_expansion_sum_zeroelim(temp16alen, temp16a,
                                               temp16blen, temp16b, temp32b);
      temp64len = fast_expansion_sum_zeroelim(temp32alen, temp32a,
                                              temp32blen, temp32b, temp64);
      finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp64len,
                                              temp64, finother);
      finswap = finnow;
      finnow = finother;
      finother = finswap;
    }
  }
  if ((bdxtail != 0.0) || (bdytail != 0.0))
  {
    if ((cdxtail != 0.0) || (cdytail != 0.0) || (adxtail != 0.0) || (adytail != 0.0))
    {
      Two_Product(cdxtail, ady, ti1, ti0);
      Two_Product(cdx, adytail, tj1, tj0);
      Two_Two_Sum(ti1, ti0, tj1, tj0, u3, u[2], u[1], u[0]);
      u[3] = u3;
      negate = -cdy;
      Two_Product(adxtail, negate, ti1, ti0);
      negate = -cdytail;
      Two_Product(adx, negate, tj1, tj0);
      Two_Two_Sum(ti1, ti0, tj1, tj0, v3, v[2], v[1], v[0]);
      v[3] = v3;
      catlen = fast_expansion_sum_zeroelim(4, u, 4, v, cat);

      Two_Product(cdxtail, adytail, ti1, ti0);
      Two_Product(adxtail, cdytail, tj1, tj0);
      Two_Two_Diff(ti1, ti0, tj1, tj0, catt3, catt[2], catt[1], catt[0]);
      catt[3] = catt3;
      cattlen = 4;
    }
    else
    {
      cat[0] = 0.0;
      catlen = 1;
      catt[0] = 0.0;
      cattlen = 1;
    }

    if (bdxtail != 0.0)
    {
      temp16alen = scale_expansion_zeroelim(bxtcalen, bxtca, bdxtail, temp16a);
      bxtcatlen = scale_expansion_zeroelim(catlen, cat, bdxtail, bxtcat);
      temp32alen = scale_expansion_zeroelim(bxtcatlen, bxtcat, 2.0 * bdx,
                                            temp32a);
      temp48len = fast_expansion_sum_zeroelim(temp16alen, temp16a,
                                              temp32alen, temp32a, temp48);
      finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp48len,
                                              temp48, finother);
      finswap = finnow;
      finnow = finother;
      finother = finswap;
      if (cdytail != 0.0)
      {
        temp8len = scale_expansion_zeroelim(4, aa, bdxtail, temp8);
        temp16alen = scale_expansion_zeroelim(temp8len, temp8, cdytail,
                                              temp16a);
        finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp16alen,
                                                temp16a, finother);
        finswap = finnow;
        finnow = finother;
        finother = finswap;
      }
      if (adytail != 0.0)
      {
        temp8len = scale_expansion_zeroelim(4, cc, -bdxtail, temp8);
        temp16alen = scale_expansion_zeroelim(temp8len, temp8, adytail,
                                              temp16a);
        finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp16alen,
                                                temp16a, finother);
        finswap = finnow;
        finnow = finother;
        finother = finswap;
      }

      temp32alen = scale_expansion_zeroelim(bxtcatlen, bxtcat, bdxtail,
                                            temp32a);
      bxtcattlen = scale_expansion_zeroelim(cattlen, catt, bdxtail, bxtcatt);
      temp16alen = scale_expansion_zeroelim(bxtcattlen, bxtcatt, 2.0 * bdx,
                                            temp16a);
      temp16blen = scale_expansion_zeroelim(bxtcattlen, bxtcatt, bdxtail,
                                            temp16b);
      temp32blen = fast_expansion_sum_zeroelim(temp16alen, temp16a,
                                               temp16blen, temp16b, temp32b);
      temp64len = fast_expansion_sum_zeroelim(temp32alen, temp32a,
                                              temp32blen, temp32b, temp64);
      finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp64len,
                                              temp64, finother);
      finswap = finnow;
      finnow = finother;
      finother = finswap;
    }
    if (bdytail != 0.0)
    {
      temp16alen = scale_expansion_zeroelim(bytcalen, bytca, bdytail, temp16a);
      bytcatlen = scale_expansion_zeroelim(catlen, cat, bdytail, bytcat);
      temp32alen = scale_expansion_zeroelim(bytcatlen, bytcat, 2.0 * bdy,
                                            temp32a);
      temp48len = fast_expansion_sum_zeroelim(temp16alen, temp16a,
                                              temp32alen, temp32a, temp48);
      finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp48len,
                                              temp48, finother);
      finswap = finnow;
      finnow = finother;
      finother = finswap;

      temp32alen = scale_expansion_zeroelim(bytcatlen, bytcat, bdytail,
                                            temp32a);
      bytcattlen = scale_expansion_zeroelim(cattlen, catt, bdytail, bytcatt);
      temp16alen = scale_expansion_zeroelim(bytcattlen, bytcatt, 2.0 * bdy,
                                            temp16a);
      temp16blen = scale_expansion_zeroelim(bytcattlen, bytcatt, bdytail,
                                            temp16b);
      temp32blen = fast_expansion_sum_zeroelim(temp16alen, temp16a,
                                               temp16blen, temp16b, temp32b);
      temp64len = fast_expansion_sum_zeroelim(temp32alen, temp32a,
                                              temp32blen, temp32b, temp64);
      finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp64len,
                                              temp64, finother);
      finswap = finnow;
      finnow = finother;
      finother = finswap;
    }
  }
  if ((cdxtail != 0.0) || (cdytail != 0.0))
  {
    if ((adxtail != 0.0) || (adytail != 0.0) || (bdxtail != 0.0) || (bdytail != 0.0))
    {
      Two_Product(adxtail, bdy, ti1, ti0);
      Two_Product(adx, bdytail, tj1, tj0);
      Two_Two_Sum(ti1, ti0, tj1, tj0, u3, u[2], u[1], u[0]);
      u[3] = u3;
      negate = -ady;
      Two_Product(bdxtail, negate, ti1, ti0);
      negate = -adytail;
      Two_Product(bdx, negate, tj1, tj0);
      Two_Two_Sum(ti1, ti0, tj1, tj0, v3, v[2], v[1], v[0]);
      v[3] = v3;
      abtlen = fast_expansion_sum_zeroelim(4, u, 4, v, abt);

      Two_Product(adxtail, bdytail, ti1, ti0);
      Two_Product(bdxtail, adytail, tj1, tj0);
      Two_Two_Diff(ti1, ti0, tj1, tj0, abtt3, abtt[2], abtt[1], abtt[0]);
      abtt[3] = abtt3;
      abttlen = 4;
    }
    else
    {
      abt[0] = 0.0;
      abtlen = 1;
      abtt[0] = 0.0;
      abttlen = 1;
    }

    if (cdxtail != 0.0)
    {
      temp16alen = scale_expansion_zeroelim(cxtablen, cxtab, cdxtail, temp16a);
      cxtabtlen = scale_expansion_zeroelim(abtlen, abt, cdxtail, cxtabt);
      temp32alen = scale_expansion_zeroelim(cxtabtlen, cxtabt, 2.0 * cdx,
                                            temp32a);
      temp48len = fast_expansion_sum_zeroelim(temp16alen, temp16a,
                                              temp32alen, temp32a, temp48);
      finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp48len,
                                              temp48, finother);
      finswap = finnow;
      finnow = finother;
      finother = finswap;
      if (adytail != 0.0)
      {
        temp8len = scale_expansion_zeroelim(4, bb, cdxtail, temp8);
        temp16alen = scale_expansion_zeroelim(temp8len, temp8, adytail,
                                              temp16a);
        finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp16alen,
                                                temp16a, finother);
        finswap = finnow;
        finnow = finother;
        finother = finswap;
      }
      if (bdytail != 0.0)
      {
        temp8len = scale_expansion_zeroelim(4, aa, -cdxtail, temp8);
        temp16alen = scale_expansion_zeroelim(temp8len, temp8, bdytail,
                                              temp16a);
        finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp16alen,
                                                temp16a, finother);
        finswap = finnow;
        finnow = finother;
        finother = finswap;
      }

      temp32alen = scale_expansion_zeroelim(cxtabtlen, cxtabt, cdxtail,
                                            temp32a);
      cxtabttlen = scale_expansion_zeroelim(abttlen, abtt, cdxtail, cxtabtt);
      temp16alen = scale_expansion_zeroelim(cxtabttlen, cxtabtt, 2.0 * cdx,
                                            temp16a);
      temp16blen = scale_expansion_zeroelim(cxtabttlen, cxtabtt, cdxtail,
                                            temp16b);
      temp32blen = fast_expansion_sum_zeroelim(temp16alen, temp16a,
                                               temp16blen, temp16b, temp32b);
      temp64len = fast_expansion_sum_zeroelim(temp32alen, temp32a,
                                              temp32blen, temp32b, temp64);
      finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp64len,
                                              temp64, finother);
      finswap = finnow;
      finnow = finother;
      finother = finswap;
    }
    if (cdytail != 0.0)
    {
      temp16alen = scale_expansion_zeroelim(cytablen, cytab, cdytail, temp16a);
      cytabtlen = scale_expansion_zeroelim(abtlen, abt, cdytail, cytabt);
      temp32alen = scale_expansion_zeroelim(cytabtlen, cytabt, 2.0 * cdy,
                                            temp32a);
      temp48len = fast_expansion_sum_zeroelim(temp16alen, temp16a,
                                              temp32alen, temp32a, temp48);
      finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp48len,
                                              temp48, finother);
      finswap = finnow;
      finnow = finother;
      finother = finswap;

      temp32alen = scale_expansion_zeroelim(cytabtlen, cytabt, cdytail,
                                            temp32a);
      cytabttlen = scale_expansion_zeroelim(abttlen, abtt, cdytail, cytabtt);
      temp16alen = scale_expansion_zeroelim(cytabttlen, cytabtt, 2.0 * cdy,
                                            temp16a);
      temp16blen = scale_expansion_zeroelim(cytabttlen, cytabtt, cdytail,
                                            temp16b);
      temp32blen = fast_expansion_sum_zeroelim(temp16alen, temp16a,
                                               temp16blen, temp16b, temp32b);
      temp64len = fast_expansion_sum_zeroelim(temp32alen, temp32a,
                                              temp32blen, temp32b, temp64);
      finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp64len,
                                              temp64, finother);
      finswap = finnow;
      finnow = finother;
      finother = finswap;
    }
  }

  return finnow[finlength - 1];
}

REAL incircle(REAL *pa, REAL *pb, REAL *pc, REAL *pd)
{
  REAL adx, bdx, cdx, ady, bdy, cdy;
  REAL bdxcdy, cdxbdy, cdxady, adxcdy, adxbdy, bdxady;
  REAL alift, blift, clift;
  REAL det;
  REAL permanent, errbound;

  adx = pa[0] - pd[0];
  bdx = pb[0] - pd[0];
  cdx = pc[0] - pd[0];
  ady = pa[1] - pd[1];
  bdy = pb[1] - pd[1];
  cdy = pc[1] - pd[1];

  bdxcdy = bdx * cdy;
  cdxbdy = cdx * bdy;
  alift = adx * adx + ady * ady;

  cdxady = cdx * ady;
  adxcdy = adx * cdy;
  blift = bdx * bdx + bdy * bdy;

  adxbdy = adx * bdy;
  bdxady = bdx * ady;
  clift = cdx * cdx + cdy * cdy;

  det = alift * (bdxcdy - cdxbdy) + blift * (cdxady - adxcdy) + clift * (adxbdy - bdxady);

  permanent = (Absolute(bdxcdy) + Absolute(cdxbdy)) * alift + (Absolute(cdxady) + Absolute(adxcdy)) * blift + (Absolute(adxbdy) + Absolute(bdxady)) * clift;
  errbound = iccerrboundA * permanent;
  if ((det > errbound) || (-det > errbound))
  {
    return det;
  }

  return incircleadapt(pa, pb, pc, pd, permanent);
}

#endif