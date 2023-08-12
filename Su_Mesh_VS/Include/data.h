/*
 * Su_Mesh数据库，包括全局变量与所有类名的声明
 */

// #pragma once
#ifndef _DATA_H
// 如果一个文件包含了此头文件多次，使用这种方法，即在第一次编译时没有定义xxx的宏，执行了下面的所有，第二次再遇到编译此文件时xxx已经被定义，就不会再编译
//  使用条件编译可以避免重复编译
//  需要注意的是要使用#ifndef语句
#define _DATA_H

// 引入CGAL库
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/intersections.h>

// 声明CGAL命名空间
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_3 Point_3;
typedef K::Segment_3 Segment_3;
typedef K::Triangle_3 Triangle_3;
typedef K::Tetrahedron_3 Tetrahedron_3;
typedef K::Sphere_3 Sphere_3;

// 声明全局变量
#define DIM 3                                             // 声明维度DIM为常值 3
extern const double Pi;                                   // 声明圆周率 3.14159265358979
extern const double Delta;                                // 声明函数求导时的初始尝试delta
extern const double tolar;                                // 声明函数求导时的两次步长间导数值容忍的最大误差
extern const double Minimum_Shape_Quality_SliverRemoval;  // 网格单元质量的极值，质量小于该值的网格单元被认定为广义薄元
extern const double Minimum_Shape_Quality_Smoothing;      // 网格单元质量的极值，质量小于该值的网格单元运行节点光顺
extern const double Minimum_Shape_Quality_Face_Transform; // 网格单元质量的极值，质量小于该值的网格单元运行面交换
extern const double Delta_volume;                         // 声明面交换合法性判断时，前后交换域总体积的最大误差值
extern const double Min_step;                             // 最小步长，基于优化的光顺内使用
extern const int Max_iter;                                // 最大迭代次数，基于优化的光顺内使用
extern const double Min_imp;                              // 最小差距值，基于优化的光顺使用，比较两次迭代后，优化域目标函数的值变换，小于该值则退出循环，接受坐标
extern const double Min_gradient;                         // 最小梯度值，梯度向量的模小于该值时代表达到极值点处，停止迭代
extern const double Min_steepest_descent;                 // 最小最速下降方向值，最速下降方向向量的模小于该值时代表达到极值点处，停止迭代
extern const double c1;                                   // Armijo准则的常数
extern const double c2;                                   // Wolfe准则的常数

// extern const int DIM;   // 声明维度DIM为常值 3
// #define Pi; // 声明圆周率 3.14159265358979

// 声明所有类名
class _SU_MESH;
class ELEM;
class NODE;
class FACE;
class EDGE;
class Point;
class Pathl;
class Setl;
class _SHEWCHUK;
class _IOS;
class _INSERT_POINT;
class _CAVITY;
class _BOUNDARY_POINT;
class _DATA_PROCESS;
class _QUALITY;
class _MESH_PROCESS;

#endif