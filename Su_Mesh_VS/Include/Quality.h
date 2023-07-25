/*
 * 声明与质量优化相关的各个算法
 */

 // #pragma once
#ifndef _QUALITY_H
#define _QUALITY_H

#include <vector>
#include <string>
#include "data.h"

class _QUALITY
{
public:
	// 单元度量准则，半径比ρ准则，给定四面体四个顶点位置，返回四面体单元形状质量，当且仅当四面体为正四面体时，quality值为1，其他任意四面体的quality值小于1
	double Shape_Quality(NODE node_A, NODE node_B, NODE node_C, NODE node_D);
	// 计算各初始网格单元质量并保存
	void Calculate_Shape_Quality(_SU_MESH* su_mesh);
	// 给定薄元的四个顶点，判断哪个点是点D，返回值是点D在对应薄元单元的form中位置
	int Check_Sliver_NodeD(NODE node_A, NODE node_B, NODE node_C, NODE node_D);
	// 给定薄元的四个顶点，返回薄元类型，分别是Cap、Sliver、Spade、Wedge四个单元类型
	std::string Check_Sliver_type(NODE node_A, NODE node_B, NODE node_C, NODE node_D);
	// 分解Cap类型薄元
	bool Decompose_Cap(_SU_MESH* su_mesh, int elemNum);
	// 分解Sliver类型薄元
	bool Decompose_Sliver(_SU_MESH* su_mesh, int elemNum);
	// 分解Sliver类型薄元的具体实现方式，adjacent_cnt==0，没有相邻网格单元
	bool Decompose_Sliver_adjacent_cnt0(_SU_MESH* su_mesh, int elemNum);
	// 分解Sliver类型薄元的具体实现方式，adjacent_cnt==1，只有一组相邻网格单元
	bool Decompose_Sliver_adjacent_cnt1(_SU_MESH* su_mesh, int elemNum, int elem_in_SliverNeig[2]);
	// 分解Sliver类型薄元的具体实现方式，adjacent_cnt==2，有两组相邻网格单元
	bool Decompose_Sliver_adjacent_cnt2(_SU_MESH* su_mesh, int elemNum, int elem_in_SliverNeig[2][2]);
	// 分解Sliver类型薄元的具体实现方式，adjacent_cnt==3，有三组相邻网格单元
	bool Decompose_Sliver_adjacent_cnt3(_SU_MESH* su_mesh, int elemNum, int elem_in_SliverNeig[3][2]);
	// 分解Spade类型薄元
	bool Decompose_Spade(_SU_MESH* su_mesh, int elemNum);
	// 分解Wedge类型薄元
	bool Decompose_Wedge(_SU_MESH* su_mesh, int elemNum);
	// 错误函数，对单元度量准则取倒数并考虑边质量约束
	double error_Function(NODE node_A, NODE node_B, NODE node_C, NODE node_D);
	// 目标函数，采用优化域所有单元的错误函数相加的方法
	double target_Function(_SU_MESH* su_mesh, NODE node_tp, std::vector<FACE> face_Optimization_Domain);
	// 精度为O(h2)的目标函数一阶导数中心差分公式
	double target_Function_h2_first(_SU_MESH* su_mesh, NODE node_tp, std::vector<FACE> face_Optimization_Domain, double delta_Value, NODE node_Delta);
	// 精度为O(h4)的目标函数一阶导数中心差分公式
	double target_Function_h4_first(_SU_MESH* su_mesh, NODE node_tp, std::vector<FACE> face_Optimization_Domain, double delta_Value, NODE node_Delta);
	// 目标函数的一阶数值导数，自变量为节点node_tp的坐标
	double target_Function_first_derivative(_SU_MESH* su_mesh, NODE node_tp, std::vector<FACE> face_Optimization_Domain, int position);
	// 精度为O(h2)的目标函数二阶导数中心差分公式
	double target_Function_h2_second(_SU_MESH* su_mesh, NODE node_tp, std::vector<FACE> face_Optimization_Domain, double delta_Value, NODE node_Delta_1, NODE node_Delta_2);
	// 精度为O(h4)的目标函数二阶导数中心差分公式
	double target_Function_h4_second(_SU_MESH* su_mesh, NODE node_tp, std::vector<FACE> face_Optimization_Domain, double delta_Value, NODE node_Delta_1, NODE node_Delta_2);
	// 目标函数的二阶数值导数，自变量为节点node_tp的坐标
	double target_Function_second_derivative(_SU_MESH* su_mesh, NODE node_tp, std::vector<FACE> face_Optimization_Domain, int position_1, int position_2);
	// 构造步长辅助函数，ϕ(α) = f(xk + αdk)，a即step步长，xk是当前点位置，dk是牛顿方向，f函数即目标函数，辅助函数节点坐标加上步长后的目标函数
	double step_helper_function(_SU_MESH* su_mesh, NODE node_tp, std::vector<FACE> face_Optimization_Domain, double steepest_descent[3][1], double step);
	// 步长辅助函数的导数，自变量为步长step，ϕ(α)' = (f'(xk + αdk))T dk，T是转置
	double step_helper_function_prime(_SU_MESH* su_mesh, NODE node_tp, std::vector<FACE> face_Optimization_Domain, double steepest_descent[3][1], double step);
	// 基于修正牛顿法，利用最优步长和最速下降方向，来求每次迭代的节点坐标变化
	NODE get_Node_Delta(double optimal_step, double steepest_descent[3][1]);
	// 用三次多项式插值法在区间内查找下一次迭代的步长值
	double cubic_interpolation_step(_SU_MESH* su_mesh, NODE node_tp, std::vector<FACE> face_Optimization_Domain, double steepest_descent[3][1], double step_LowerLimit, double step_UpperLimit);
	// 在包含最优解的区间内进一步搜索最优解
	double Zoom(_SU_MESH* su_mesh, NODE node_tp, std::vector<FACE> face_Optimization_Domain, double steepest_descent[3][1], double step_LowerLimit, double step_UpperLimit);
	// 采用强Wolfe准则，利用牛顿方向，依据修正牛顿法可以直接取初始步长为1，确定类最优步长
	double Line_Search_Algorithm_with_Wolfe(_SU_MESH* su_mesh, NODE node_tp, std::vector<FACE> face_Optimization_Domain, double steepest_descent[3][1], double Init_step);
	// 构造局部优化域，有劣质单元的所有节点来作为待优化节点
	void Construct_Local_Optimization_Domain_All(_SU_MESH* su_mesh, std::vector<int> Init_inferior_elem, std::vector<int>* Laplacian_Smoothing_nodeNum);
	// 智能Laplacian光顺，给定一个节点，实行Laplacian光顺，并且将该节点移动到其相邻节点的几何中心前，若不能提高相邻网格质量，则不予移动操作，移动成功返回true
	bool Laplacian_Smoothing(_SU_MESH* su_mesh, int Laplacian_Smoothing_nodeNum);
	// 基于优化的光顺，给定一个节点，采用对目标函数求最优解的方法确定新节点位置，目标函数为该节点相邻单元的所有质量函数的倒数相加，即优化域错误函数相加，移动成功返回true
	bool Optimization_based_Smoothing_The_all(_SU_MESH* su_mesh, int Optimization_based_Smoothing_nodeNum);
	// 输出当前三角化的网格单元质量信息
	void Quality_Information(_SU_MESH* su_mesh);
	// 利用广义薄元分解实现网格质量优化
	void Quality_Optimization_SliverRemoval(_SU_MESH* su_mesh);
	// 利用节点光顺实现网格质量优化
	void Quality_Optimization_Smoothing(_SU_MESH* su_mesh);
	// T23交换合法性判断，若不合法返回-1，若合法，返回交换后交换域内最低网格质量
	double Face_Transform_23_Legality(_SU_MESH* su_mesh, FACE face_tp, std::vector<int> elemNum_include_face);
	// T23交换
	std::vector<int> Face_Transform_23(_SU_MESH* su_mesh, FACE face_tp);
	// T32交换合法性判断，若不合法返回-1，若合法，返回交换后交换域内最低网格质量
	double Face_Transform_32_Legality(_SU_MESH* su_mesh, EDGE edge_tp, std::vector<int> elemNum_include_edge);
	// T32交换
	std::vector<int> Face_Transform_32(_SU_MESH* su_mesh, EDGE edge_tp);
	// T44交换合法性判断，若不合法返回-1，若合法，返回交换后交换域内最低网格质量
	double Face_Transform_44_Legality(_SU_MESH* su_mesh, EDGE edge_tp, std::vector<int> elemNum_include_edge);
	// T44交换
	std::vector<int> Face_Transform_44(_SU_MESH* su_mesh, EDGE edge_tp);
	// T56交换合法性判断，若不合法返回-1，若合法，返回交换后交换域内最低网格质量
	double Face_Transform_56_Legality(_SU_MESH* su_mesh, EDGE edge_tp, std::vector<int> elemNum_include_edge);
	// T56交换
	std::vector<int> Face_Transform_56(_SU_MESH* su_mesh, EDGE edge_tp);
	// 再利用面交换实现网格质量优化
	void Quality_Optimization_Face_Transform(_SU_MESH* su_mesh);

	void check(_SU_MESH* su_mesh);
	// 已发表论文中的基于优化的光顺，用于专利的对比
	bool Optimization_based_Smoothing_Compared(_SU_MESH* su_mesh, int Optimization_based_Smoothing_nodeNum);
	void Compute_HessianLike_matrice(double matrix_1[3][3], double matrix_2[3][1], double matrix_3[3][1], double matrix_target[3][3]);
private:
};

#endif