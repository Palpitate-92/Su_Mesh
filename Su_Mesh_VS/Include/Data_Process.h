/*
 * 声明与数据处理和空间几何运算相关的各种算法
 */

// #pragma once
#ifndef _DATA_PROCESS_H
#define _DATA_PROCESS_H

#include <vector>
#include "data.h"

class _DATA_PROCESS
{
public:
    // 定义一个sign函数即符号函数
    double sign(double value);
    // 给定两个节点坐标，返回节点距离
    double get_dist(double A[3], double B[3]);
    // 给定一个网格单元的四个节点，计算其重心节点
    NODE center_of_gravity(NODE node_tp1, NODE node_tp2, NODE node_tp3, NODE node_tp4);
    // 给定网格单元的四个节点与待判断节点，判断当前网格单元外接球是否包含待插入节点，返回值大于0则包含，小于0不包含，等于0则5点共球
    double in_sphere(NODE node_tp1, NODE node_tp2, NODE node_tp3, NODE node_tp4, NODE node_tp);
    // 给定三角形三个顶点位置，返回三角形面积，顶点是逆时针方向的
    double triangle_area(NODE node_A, NODE node_B, NODE node_C);
    // 利用三角形面积判断点是否在三角形内部
    bool point_internal_triangle(NODE node_A, NODE node_B, NODE node_C, NODE point);
    // 给定四面体四个顶点位置，返回该四面体有向体积，小于0代表节点顺序出错，不是orient3d()定义的正方向
    double tetrahedral_volume(NODE node_A, NODE node_B, NODE node_C, NODE node_D);
    // 给定两条边的节点坐标，判断该两条边是否相交。此处使用CGAL几何算法库
    bool Edge_Edge_Intersection(Point *intersection_point, NODE node_A, NODE node_B, NODE node_C, NODE node_D);
    // 给定一条边和一个面的节点坐标，判断是否相交。此处使用CGAL几何算法库
    bool Edge_Face_Intersection(Point *intersection_point, NODE node_A, NODE node_B, NODE node_C, NODE node_D, NODE node_E);
    // 给定一条边和一个网格单元的节点坐标，判断其交点数。此处使用CGAL几何算法库
    int Edge_Elem_Intersection(NODE node_A, NODE node_B, NODE node_C, NODE node_D, NODE node_E, NODE node_F);
    // 给定一个面和一个网格单元的节点坐标，判断其交点数。此处使用CGAL几何算法库
    int Face_Elem_Intersection(NODE node_A, NODE node_B, NODE node_C, NODE node_D, NODE node_E, NODE node_F, NODE node_G);
    // 求3x1矩阵的转置矩阵
    void Matrix_transpose(double matrix_1[3][1], double matrix_2[3]);
    // 求3x1矩阵向量的模
    double Vector_Module(double matrix[3][1]);
    // 求二阶矩阵行列式
    double Matrix_Determinant(double matrix[2][2]);
    // 求三阶矩阵行列式
    double Matrix_Determinant(double matrix[3][3]);
    // 对3x3矩阵求逆，利用伴随矩阵来求
    void Inverse_Matrix(double matrix[3][3]);
    // 求矩阵乘积，1x3与3x1矩阵乘积
    double matrix_product(double matrix_1[3], double matrix_2[3][1]);
    // 求矩阵乘积，1x3与3x3矩阵乘积
    void matrix_product(double matrix_1[3], double matrix_2[3][3], double product[3]);
    // 求矩阵乘积，3x1与1x3矩阵乘积
    void matrix_product(double matrix_1[3][1], double matrix_2[3], double product[3][3]);
    // 求矩阵乘积，3x3与3x1矩阵乘积
    void matrix_product(double matrix_1[3][3], double matrix_2[3][1], double product[3][1]);
    // 求矩阵乘积，3x3与3x3矩阵乘积
    void matrix_product(double matrix_1[3][3], double matrix_2[3][3], double product[3][3]);
    // 求矩阵乘积，1x3、3x3、3x1矩阵乘积
    double matrix_product(double matrix_1[3], double matrix_2[3][3], double matrix_3[3][1]);
    // 求矩阵乘积，3x3、3x1、1x3、3x3矩阵乘积
    void matrix_product(double matrix_1[3][3], double matrix_2[3][1], double matrix_3[3], double matrix_4[3][3], double product[3][3]);
    // 判断hessian矩阵是否正定，若不正定则修改其值使其正定，此处是使hessian矩阵加上单位矩阵的常数倍
    void Positivity_Hessian_Matrix(double hessian_matrix[3][3]);

private:
};

#endif