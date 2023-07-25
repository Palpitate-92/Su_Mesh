#include "Su_Mesh.h"

const double delta_1 = 1e-3;
const double delta_2 = 1e-2;

double _QUALITY::Shape_Quality(NODE node_A, NODE node_B, NODE node_C, NODE node_D)
{
	_DATA_PROCESS data_process;
	double volume = data_process.tetrahedral_volume(node_A, node_B, node_C, node_D);
	// 分别求四面体四个面面积
	double area_1 = data_process.triangle_area(node_A, node_B, node_C);
	double area_2 = data_process.triangle_area(node_A, node_B, node_D);
	double area_3 = data_process.triangle_area(node_A, node_C, node_D);
	double area_4 = data_process.triangle_area(node_B, node_C, node_D);
	// 分别求四面体六条边长度
	double l[] = { data_process.get_dist(node_A.pos, node_B.pos),
				  data_process.get_dist(node_A.pos, node_C.pos),
				  data_process.get_dist(node_A.pos, node_D.pos),
				  data_process.get_dist(node_B.pos, node_C.pos),
				  data_process.get_dist(node_B.pos, node_D.pos),
				  data_process.get_dist(node_C.pos, node_D.pos) };
	std::sort(l, l + 6); // 对六条边长度进行升序排序
	double quality = 6 * sqrt(6) * volume / ((area_1 + area_2 + area_3 + area_4) * l[5]);
	return quality;
}

void _QUALITY::Calculate_Shape_Quality(_SU_MESH* su_mesh)
{
	// 计算各初始网格单元质量
	for (std::vector<ELEM>::iterator elem_iter = su_mesh->elem.begin();
		elem_iter != su_mesh->elem.end();
		++elem_iter)
		elem_iter->quality = abs(Shape_Quality(su_mesh->node.at(elem_iter->form[0]),
			su_mesh->node.at(elem_iter->form[1]),
			su_mesh->node.at(elem_iter->form[2]),
			su_mesh->node.at(elem_iter->form[3])));
	return;
}

int _QUALITY::Check_Sliver_NodeD(NODE node_A, NODE node_B, NODE node_C, NODE node_D)
{
	_DATA_PROCESS data_process;
	// 分别求四面体四个面面积
	double area[] = { data_process.triangle_area(node_B, node_C, node_D),
					 data_process.triangle_area(node_A, node_C, node_D),
					 data_process.triangle_area(node_A, node_B, node_D),
					 data_process.triangle_area(node_A, node_B, node_C) };
	// 找到面积最大的三角形，该三角形相对的节点即是点D
	double area_max = std::max(std::max(area[0], area[1]), std::max(area[2], area[3]));
	if (area[0] == area_max)
		return 0;
	else if (area[1] == area_max)
		return 1;
	else if (area[2] == area_max)
		return 2;
	else
		return 3;
}

std::string _QUALITY::Check_Sliver_type(NODE node_A, NODE node_B, NODE node_C, NODE node_D)
{
	_DATA_PROCESS data_process;
	NODE node_tp[] = { node_A, node_B, node_C, node_D };
	// 分别求四面体四个面面积
	double area[] = { data_process.triangle_area(node_B, node_C, node_D),
					 data_process.triangle_area(node_A, node_C, node_D),
					 data_process.triangle_area(node_A, node_B, node_D),
					 data_process.triangle_area(node_A, node_B, node_C) };
	// 找到面积最大的三角形，并将该三角形相对的节点替换到node_tp[3]
	double area_check[] = { area[0], area[1], area[2], area[3] };
	std::sort(area_check, area_check + 4);
	for (int i = 0; i < 4; i++)
		if (area[i] == area_check[3] && i != 3)
			std::swap(node_tp[i], node_tp[3]);
	double EPSArea = 1e-6 * (area_check[0] + area_check[1] + area_check[2]) / 3.0;
	double distance[] = { data_process.get_dist(node_tp[0].pos, node_tp[3].pos),
						 data_process.get_dist(node_tp[1].pos, node_tp[3].pos),
						 data_process.get_dist(node_tp[2].pos, node_tp[3].pos) };
	std::sort(distance, distance + 3);
	double EPSDistance = 1e-6 * (distance[0] + distance[1] + distance[2]) / 3.0;
	if (area_check[0] < EPSArea)
	{
		if (distance[0] < EPSDistance)
			return "Wedge";
		else
			return "Spade";
	}
	else
	{
		if (area_check[0] + area_check[1] + area_check[2] == area_check[3])
			return "Cap";
		else
			return "Sliver";
	}
}

bool _QUALITY::Decompose_Cap(_SU_MESH* su_mesh, int elemNum)
{
	return true;
}

bool _QUALITY::Decompose_Sliver(_SU_MESH* su_mesh, int elemNum)
{
	// 首先判断当前薄元的4个相邻网格单元的相邻情况，一共要进行6次判断
	// 若当前判断的两个网格单元相邻，则adjacent_cnt+1，并且记录下当前两个网格单元在薄元的neig位置，并且在任何情况下，adjacent_cnt都不可能大于3
	int adjacent_cnt = 0,
		elem_in_SliverNeig[3][2] = { {-1, -1}, {-1, -1}, {-1, -1} };
	ELEM elem_Sliver = su_mesh->elem.at(elemNum); // 储存当前薄元
	_MESH_PROCESS mesh_process;
	for (int i = 0; i < DIM + 1; i++)
	{
		if (elem_Sliver.neig[i] == -1)
			continue;
		for (int j = i + 1; j < DIM + 1; j++)
		{
			if (elem_Sliver.neig[j] == -1)
				continue;
			if (mesh_process.AdjacentElem_pos(su_mesh->elem.at(elem_Sliver.neig[i]), elem_Sliver.neig[j]) != -1)
			{
				// 在任何情况下，adjacent_cnt都不可能大于3
				if (adjacent_cnt == 3)
				{
					std::cout << "Incorrect mesh found when decomposing thin \"Sliver\" elements!\n";
					exit(-1);
				}
				elem_in_SliverNeig[adjacent_cnt][0] = i;
				elem_in_SliverNeig[adjacent_cnt][1] = j;
				adjacent_cnt++;
			}
		}
	}
	// 用cnt值来判断接下来该怎么去分解当前薄元
	switch (adjacent_cnt)
	{
	case 0:
		Decompose_Sliver_adjacent_cnt0(su_mesh, elemNum);
		break;
	case 1:
		Decompose_Sliver_adjacent_cnt1(su_mesh, elemNum, elem_in_SliverNeig[0]);
		break;
	case 2:
		Decompose_Sliver_adjacent_cnt2(su_mesh, elemNum, elem_in_SliverNeig);
		break;
	case 3:
		Decompose_Sliver_adjacent_cnt3(su_mesh, elemNum, elem_in_SliverNeig);
		break;
	default:
		std::cout << "Incorrect mesh found when decomposing thin \"Sliver\" "
			"elements!\n";
		exit(-1);
		break;
	}
	return true;
}

bool _QUALITY::Decompose_Sliver_adjacent_cnt0(_SU_MESH* su_mesh, int elemNum)
{
	return true;
}

bool _QUALITY::Decompose_Sliver_adjacent_cnt1(_SU_MESH* su_mesh, int elemNum, int elem_in_SliverNeig[2])
{
	ELEM elem_Sliver = su_mesh->elem.at(elemNum); // 储存当前薄元
	// 储存当前薄元待局部变换的相邻单元编号
	int elemNum_tp[] = { elem_Sliver.neig[elem_in_SliverNeig[0]],
						elem_Sliver.neig[elem_in_SliverNeig[1]] };
	// 储存当前薄元待局部变换的相邻单元
	ELEM elem_adjacent[] = { su_mesh->elem.at(elemNum_tp[0]), su_mesh->elem.at(elemNum_tp[1]) };
	_MESH_PROCESS mesh_process;
	_BOUNDARY_RECOVERY boundary_recovery;
	// 储存当前薄元待局部变换的相邻单元的有效相邻单元，即去除这两个单元自身，和薄元单元，最终会得到四个相邻单元编号并储存其相邻网格面，两个容器一一对应
	std::vector<int> elemNum_adjacent;
	int elemNum_iter;
	std::vector<FACE> face_adjacent;
	std::vector<FACE>::iterator face_iter;
	for (int i = 0; i < 2; i++)
		for (int j = 0; j < DIM + 1; j++)
		{
			if (elem_adjacent[i].neig[j] != elemNum && elem_adjacent[i].neig[j] != elemNum_tp[0] && elem_adjacent[i].neig[j] != elemNum_tp[1])
			{
				elemNum_adjacent.push_back(elem_adjacent[i].neig[j]);
				face_adjacent.push_back(mesh_process.Node_Opposite_Face(elem_adjacent[i], elem_adjacent[i].form[j]));
			}
		}
	// 储存相邻单元在薄元外的公共节点
	int nodeNum_public = elem_adjacent[0].form[mesh_process.AdjacentElem_pos(elem_adjacent[0], elemNum)];
	EDGE edge_tp1 = { elem_Sliver.form[elem_in_SliverNeig[0]],
					 elem_Sliver.form[elem_in_SliverNeig[1]] };
	// 储存相邻单元在薄元内的公共边
	EDGE edge_tp2 = mesh_process.Edge_Opposite_Edge(elem_Sliver, edge_tp1);
	// 储存两个将要生成的网格单元，并储存直接能得到的相邻信息，需要后续进行再处理的相邻信息先赋值为-1
	ELEM elem_tp[] = {
		{nodeNum_public, edge_tp2.form[0], edge_tp1.form[0], edge_tp1.form[1], elem_Sliver.neig[mesh_process.ELEM_Include_Node(elem_Sliver, edge_tp2.form[1])], elemNum_tp[1], -1, -1},
		{nodeNum_public, edge_tp2.form[1], edge_tp1.form[0], edge_tp1.form[1], elem_Sliver.neig[mesh_process.ELEM_Include_Node(elem_Sliver, edge_tp2.form[0])], elemNum_tp[0], -1, -1} };
	FACE face_tp[] = { {nodeNum_public, edge_tp2.form[0], edge_tp1.form[1]},
					  {nodeNum_public, edge_tp2.form[0], edge_tp1.form[0]},
					  {nodeNum_public, edge_tp2.form[1], edge_tp1.form[1]},
					  {nodeNum_public, edge_tp2.form[1], edge_tp1.form[0]} };
	for (int i = 0; i < 4; i++)
		face_tp[i].Sort();
	elem_tp[0].neig[2] = elemNum_adjacent.at(
		elemNum_iter = std::distance(face_adjacent.begin(), face_iter = std::find(face_adjacent.begin(), face_adjacent.end(), face_tp[0])));
	elem_tp[0].neig[3] = elemNum_adjacent.at(
		elemNum_iter = std::distance(face_adjacent.begin(), face_iter = std::find(face_adjacent.begin(), face_adjacent.end(), face_tp[1])));
	elem_tp[1].neig[2] = elemNum_adjacent.at(
		elemNum_iter = std::distance(face_adjacent.begin(), face_iter = std::find(face_adjacent.begin(), face_adjacent.end(), face_tp[2])));
	elem_tp[1].neig[3] = elemNum_adjacent.at(
		elemNum_iter = std::distance(face_adjacent.begin(), face_iter = std::find(face_adjacent.begin(), face_adjacent.end(), face_tp[3])));
	elem_tp[0].Sort();
	elem_tp[1].Sort();
	// 替换elem容器内值前，将广义薄元与容器内最后一个元素交换位置，然后在容器内删除该元素，实现广义薄元的删除
	boundary_recovery.ReplaceElem_two(su_mesh, elemNum, su_mesh->elem_num - 1);
	boundary_recovery.Removal_LastElem(su_mesh, su_mesh->elem_num - 1);
	su_mesh->elem.at(elemNum_tp[0]) = elem_tp[0];
	su_mesh->elem.at(elemNum_tp[1]) = elem_tp[1];
	int value_tp;
	// 修改所有相邻信息
	for (int i = 0; i < 2; i++)
	{
		for (int j = 0; j < DIM + 1; j++)
		{
			if (elem_tp[i].neig[j] == -1)
				continue;
			if (elem_tp[i].neig[j] == su_mesh->elem_num)
			{
				elem_tp[i].neig[j] = elemNum;
				su_mesh->elem.at(elemNum_tp[i]).neig[j] = elemNum;
			}
			value_tp = mesh_process.Face_Opposite_Node(su_mesh->elem.at(elem_tp[i].neig[j]), mesh_process.Node_Opposite_Face(elem_tp[i], elem_tp[i].form[j]));
			if (value_tp == -1)
			{
				std::cout << "Decompose_Sliver_adjacent_cnt1 run error, please "
					"check the program!\n";
				exit(-1);
			}
			su_mesh->elem.at(elem_tp[i].neig[j]).neig[value_tp] = elemNum_tp[i];
		}
		// 修改单元所有节点的elem值
		mesh_process.Renew_NodeElem(su_mesh, elemNum_tp[i]);
		// 修改单元质量
		su_mesh->elem.at(elemNum_tp[i]).quality = abs(Shape_Quality(su_mesh->node.at(su_mesh->elem.at(elemNum_tp[i]).form[0]),
			su_mesh->node.at(su_mesh->elem.at(elemNum_tp[i]).form[1]),
			su_mesh->node.at(su_mesh->elem.at(elemNum_tp[i]).form[2]),
			su_mesh->node.at(su_mesh->elem.at(elemNum_tp[i]).form[3])));
	}
	return true;
}

bool _QUALITY::Decompose_Sliver_adjacent_cnt2(_SU_MESH* su_mesh, int elemNum, int elem_in_SliverNeig[2][2])
{
	// adjacent_cnt==2的情况下，其实就是两个adjacent_cnt==1的情况
	// 只需要保留对网格质量提升最显著的结果，即比较每组相邻单元进行局部变换前后的平均质量，最小质量
	ELEM elem_Sliver = su_mesh->elem.at(elemNum); // 储存当前薄元
	// 储存当前薄元的相邻单元
	ELEM elem_adjacent[] = { su_mesh->elem.at(elem_Sliver.neig[elem_in_SliverNeig[0][0]]),
							su_mesh->elem.at(elem_Sliver.neig[elem_in_SliverNeig[0][1]]),
							su_mesh->elem.at(elem_Sliver.neig[elem_in_SliverNeig[1][0]]),
							su_mesh->elem.at(elem_Sliver.neig[elem_in_SliverNeig[1][1]]) };
	// 储存两组相邻单元进行局部变换前质量，并且使其从小到大排序
	double Shape_Quality_before[2][2] = { {elem_adjacent[0].quality, elem_adjacent[1].quality},
										 {elem_adjacent[2].quality, elem_adjacent[3].quality} };
	if (Shape_Quality_before[0][0] > Shape_Quality_before[0][1])
		std::swap(Shape_Quality_before[0][0], Shape_Quality_before[0][1]);
	if (Shape_Quality_before[1][0] > Shape_Quality_before[1][1])
		std::swap(Shape_Quality_before[1][0], Shape_Quality_before[1][1]);
	// 储存两组相邻单元进行局部变换前平均质量
	double Shape_Quality_average_before[2] = { (Shape_Quality_before[0][0] + Shape_Quality_before[0][1]) / 2,
											  (Shape_Quality_before[1][0] + Shape_Quality_before[1][1]) / 2 };
	double Shape_Quality_after[2][2];      // 储存两组相邻单元进行局部变换后质量
	double Shape_Quality_average_after[2]; // 储存两组相邻单元进行局部变换后平均质量
	int nodeNum_public;                    // 储存相邻单元在薄元外的公共节点
	EDGE edge_tp1, edge_tp2;
	_MESH_PROCESS mesh_process;
	bool judge;
	// 计算两组相邻单元进行局部变换后质量信息
	for (int i = 0; i < 2; i++)
	{
		nodeNum_public = elem_adjacent[i * 2].form[mesh_process.AdjacentElem_pos(elem_adjacent[i * 2], elemNum)];
		edge_tp1.form[0] = elem_Sliver.form[elem_in_SliverNeig[i][0]];
		edge_tp1.form[1] = elem_Sliver.form[elem_in_SliverNeig[i][1]];
		edge_tp2 = mesh_process.Edge_Opposite_Edge(elem_Sliver, edge_tp1);
		Shape_Quality_after[i][0] = abs(Shape_Quality(su_mesh->node.at(nodeNum_public),
			su_mesh->node.at(edge_tp1.form[0]),
			su_mesh->node.at(edge_tp1.form[1]),
			su_mesh->node.at(edge_tp2.form[0])));
		Shape_Quality_after[i][1] = abs(Shape_Quality(su_mesh->node.at(nodeNum_public),
			su_mesh->node.at(edge_tp1.form[0]),
			su_mesh->node.at(edge_tp1.form[1]),
			su_mesh->node.at(edge_tp2.form[1])));
		if (Shape_Quality_after[i][0] > Shape_Quality_after[i][1])
			std::swap(Shape_Quality_after[i][0], Shape_Quality_after[i][1]);
		Shape_Quality_average_after[i] = (Shape_Quality_after[i][0] + Shape_Quality_after[i][1]) / 2;
	}
	// 判断两组相邻单元进行局部优化前后质量信息的变化情况，选择对网格质量提升最显著的结果进行局部变换
	// 一般情况下首先判断能否提升最低质量，选择能提升最低质量的那组相邻单元
	if (Shape_Quality_after[0][0] > Shape_Quality_before[0][0] && Shape_Quality_after[1][0] < Shape_Quality_before[1][0])
		judge = Decompose_Sliver_adjacent_cnt1(su_mesh, elemNum, elem_in_SliverNeig[0]);
	else if (Shape_Quality_after[0][0] < Shape_Quality_before[0][0] && Shape_Quality_after[1][0] > Shape_Quality_before[1][0])
		judge = Decompose_Sliver_adjacent_cnt1(su_mesh, elemNum, elem_in_SliverNeig[1]);
	// 接下来，则根据平均质量的提升幅度来选择
	else
	{
		if (Shape_Quality_average_after[0] - Shape_Quality_average_before[0] >
			Shape_Quality_average_after[1] - Shape_Quality_average_before[1])
			judge = Decompose_Sliver_adjacent_cnt1(su_mesh, elemNum, elem_in_SliverNeig[0]);
		else
			judge = Decompose_Sliver_adjacent_cnt1(su_mesh, elemNum, elem_in_SliverNeig[1]);
	}
	return judge;
}

bool _QUALITY::Decompose_Sliver_adjacent_cnt3(_SU_MESH* su_mesh, int elemNum, int elem_in_SliverNeig[3][2])
{
	return true;
}

bool _QUALITY::Decompose_Spade(_SU_MESH* su_mesh, int elemNum)
{
	return true;
}

bool _QUALITY::Decompose_Wedge(_SU_MESH* su_mesh, int elemNum)
{
	return true;
}

double _QUALITY::error_Function(NODE node_A, NODE node_B, NODE node_C, NODE node_D)
{
	// _DATA_PROCESS data_process;
	// double l[] = {data_process.get_dist(node_A.pos, node_B.pos),
	//               data_process.get_dist(node_A.pos, node_C.pos),
	//               data_process.get_dist(node_A.pos, node_D.pos),
	//               data_process.get_dist(node_B.pos, node_C.pos),
	//               data_process.get_dist(node_B.pos, node_D.pos),
	//               data_process.get_dist(node_C.pos, node_D.pos)};
	// std::sort(l, l + 6); // 对六条边长度进行升序排序
	// // 考虑边质量约束
	// double edge_constraints = 0;
	// for (int i = 0; i < 6; i++)
	//   edge_constraints += pow(l[i] / l[0] - l[0] / l[i], 2);
	double quality = 1 / Shape_Quality(node_A, node_B, node_C, node_D);
	//double quality = Shape_Quality(node_A, node_B, node_C, node_D);
	if (quality < 0)
		return -1;
	return quality; //+ edge_constraints;
}

double _QUALITY::target_Function(_SU_MESH* su_mesh, NODE node_tp, std::vector<FACE> face_Optimization_Domain)
{
	double target_Function_value = 0; // 定义一个变量，储存目标函数值
	double value_tp;                  // 声明一个变量，临时储存当前判断单元错误函数值
	// 遍历face_Optimization_Domain容器，累加所有单元错误函数值
	for (std::vector<FACE>::iterator face_iter = face_Optimization_Domain.begin();
		face_iter != face_Optimization_Domain.end();
		++face_iter)
	{
		value_tp = error_Function(su_mesh->node.at(face_iter->form[0]), su_mesh->node.at(face_iter->form[1]), su_mesh->node.at(face_iter->form[2]), node_tp);
		if (value_tp < 0)
			return -1;
		//target_Function_value = target_Function_value > value_tp ? value_tp : target_Function_value;
		target_Function_value += value_tp;
	}
	return target_Function_value;
}

double _QUALITY::target_Function_h2_first(_SU_MESH* su_mesh, NODE node_tp, std::vector<FACE> face_Optimization_Domain, double delta_Value, NODE node_Delta)
{
	double a = target_Function(su_mesh, node_tp + node_Delta, face_Optimization_Domain);
	double b = target_Function(su_mesh, node_tp - node_Delta, face_Optimization_Domain);
	if (a == -1 || b == -1)
		return -1;
	double value = (a - b) / (2 * delta_Value);
	return value;
}

double _QUALITY::target_Function_h4_first(_SU_MESH* su_mesh, NODE node_tp, std::vector<FACE> face_Optimization_Domain, double delta_Value, NODE node_Delta)
{
	double a = target_Function(su_mesh, node_tp + node_Delta * 2.0, face_Optimization_Domain);
	double b = target_Function(su_mesh, node_tp + node_Delta, face_Optimization_Domain);
	double c = target_Function(su_mesh, node_tp - node_Delta, face_Optimization_Domain);
	double d = target_Function(su_mesh, node_tp - node_Delta * 2.0, face_Optimization_Domain);
	if (a == -1 || b == -1 || c == -1 || d == -1)
		return -1;
	double value = (-a + 8 * b - 8 * c + d) / (12 * delta_Value);
	return value;
}

double _QUALITY::target_Function_first_derivative(_SU_MESH* su_mesh, NODE node_tp, std::vector<FACE> face_Optimization_Domain, int position)
{
	double value[10], error_bounds[10];
	double delta_Value = Delta;
	//double delta_Value = Delta * delta_1;
	NODE node_Delta = { 0, 0, 0 };
	int cnt = 0;
	node_Delta.pos[position] = delta_Value;
	if ((value[cnt] = target_Function_h4_first(su_mesh, node_tp, face_Optimization_Domain, delta_Value, node_Delta)) == -1)
		value[cnt] = -1;
	//return value[cnt];
	// return -1;
	error_bounds[cnt] = 1;
	while (error_bounds[cnt] > tolar && cnt < 9)
	{
		cnt++;
		delta_Value /= 10;
		node_Delta.pos[position] = delta_Value;
		if ((value[cnt] = target_Function_h4_first(su_mesh, node_tp, face_Optimization_Domain, delta_Value, node_Delta)) == -1)
			value[cnt] = 0;
		// return -1;
		error_bounds[cnt] = abs(value[cnt] - value[cnt - 1]);
	}
	if (cnt == 9)
	{
		double error_bounds_judge[10];
		std::copy(error_bounds, error_bounds + 10, error_bounds_judge);
		std::sort(error_bounds_judge, error_bounds_judge + 10);
		for (int i = 0; i < 10; i++)
			if (error_bounds[i] == error_bounds_judge[0])
				return value[i];
	}
	return value[cnt];
}

double _QUALITY::target_Function_h2_second(_SU_MESH* su_mesh, NODE node_tp, std::vector<FACE> face_Optimization_Domain, double delta_Value, NODE node_Delta_1, NODE node_Delta_2)
{
	double a = target_Function(su_mesh, node_tp + node_Delta_1 + node_Delta_2, face_Optimization_Domain);
	double b = target_Function(su_mesh, node_tp + node_Delta_1 - node_Delta_2, face_Optimization_Domain);
	double c = target_Function(su_mesh, node_tp - node_Delta_1 + node_Delta_2, face_Optimization_Domain);
	double d = target_Function(su_mesh, node_tp - node_Delta_1 - node_Delta_2, face_Optimization_Domain);
	if (a == -1 || b == -1 || c == -1 || d == -1)
		return -1;
	double value = (a - b - c + d) / (4 * delta_Value * delta_Value);
	return value;
}

double _QUALITY::target_Function_h4_second(_SU_MESH* su_mesh, NODE node_tp, std::vector<FACE> face_Optimization_Domain, double delta_Value, NODE node_Delta_1, NODE node_Delta_2)
{
	return -1;
}

double _QUALITY::target_Function_second_derivative(_SU_MESH* su_mesh, NODE node_tp, std::vector<FACE> face_Optimization_Domain, int position_1, int position_2)
{
	double value[10], error_bounds[10];
	double delta_Value = Delta;
	//double delta_Value = Delta * delta_2;
	NODE node_Delta[2] = { {0, 0, 0}, {0, 0, 0} };
	int cnt = 0;
	node_Delta[0].pos[position_1] = delta_Value;
	node_Delta[1].pos[position_2] = delta_Value;
	if ((value[cnt] = target_Function_h2_second(su_mesh, node_tp, face_Optimization_Domain, delta_Value, node_Delta[0], node_Delta[1])) == -1)
		value[cnt] = -1;
	//return value[cnt];
	// return -1;
	error_bounds[cnt] = 1;
	while (error_bounds[cnt] > tolar && cnt < 9)
	{
		cnt++;
		delta_Value /= 10;
		node_Delta[0].pos[position_1] = delta_Value;
		node_Delta[1].pos[position_2] = delta_Value;
		if ((value[cnt] = target_Function_h2_second(su_mesh, node_tp, face_Optimization_Domain, delta_Value, node_Delta[0], node_Delta[1])) == -1)
			value[cnt] = 0;
		// return -1;
		error_bounds[cnt] = abs(value[cnt] - value[cnt - 1]);
	}
	if (cnt == 9)
	{
		double error_bounds_judge[10];
		std::copy(error_bounds, error_bounds + 10, error_bounds_judge);
		std::sort(error_bounds_judge, error_bounds_judge + 10);
		for (int i = 0; i < 10; i++)
			if (error_bounds[i] == error_bounds_judge[0])
				return value[i];
	}
	return value[cnt];
}

double _QUALITY::step_helper_function(_SU_MESH* su_mesh, NODE node_tp, std::vector<FACE> face_Optimization_Domain, double steepest_descent[3][1], double step)
{
	double steepest_value
		[3][1]; // 声明一个变量，作为最速下降值，即步长乘上最速下降方向
	steepest_value[0][0] = step * steepest_descent[0][0];
	steepest_value[1][0] = step * steepest_descent[1][0];
	steepest_value[2][0] = step * steepest_descent[2][0];
	return target_Function(su_mesh, node_tp + steepest_value, face_Optimization_Domain);
}

double _QUALITY::step_helper_function_prime(_SU_MESH* su_mesh, NODE node_tp, std::vector<FACE> face_Optimization_Domain, double steepest_descent[3][1], double step)
{
	_DATA_PROCESS data_process;
	// 步长辅助函数对step求导为ϕ(α)' = (f'(xk + αdk))T
	// dk，而等式右边第一项其实就是移动node_tp位置，然后在移动后的位置处对node_tp求导
	// 所以可以先移动node_tp位置，然后直接使用目标函数的求导函数直接对其进行求导
	for (int i = 0; i < DIM; i++)
		node_tp.pos[i] += step * steepest_descent[i][0];
	double step_helper_function_prime_value[3] = { target_Function_first_derivative(su_mesh, node_tp, face_Optimization_Domain, 0),
												  target_Function_first_derivative(su_mesh, node_tp, face_Optimization_Domain, 1),
												  target_Function_first_derivative(su_mesh, node_tp, face_Optimization_Domain, 2) };
	return data_process.matrix_product(step_helper_function_prime_value,
		steepest_descent);
}

NODE _QUALITY::get_Node_Delta(double optimal_step, double steepest_descent[3][1])
{
	NODE node_tp; // 声明一个NODE类，临时储存节点坐标变化
	node_tp.pos[0] = optimal_step * steepest_descent[0][0];
	node_tp.pos[1] = optimal_step * steepest_descent[1][0];
	node_tp.pos[2] = optimal_step * steepest_descent[2][0];
	return node_tp;
}

double _QUALITY::cubic_interpolation_step(_SU_MESH* su_mesh, NODE node_tp, std::vector<FACE> face_Optimization_Domain, double steepest_descent[3][1], double step_LowerLimit, double step_UpperLimit)
{
	_DATA_PROCESS data_process;
	if (step_LowerLimit > step_UpperLimit)
		std::swap(step_LowerLimit, step_UpperLimit);
	// 定义四个变量，分别储存区间上下限的辅助函数与辅助函数导数，作为三次多项式插值的端点条件
	double helper_function_step_Lower = step_helper_function(su_mesh, node_tp, face_Optimization_Domain, steepest_descent, step_LowerLimit);
	double helper_function_step_Lower_prime = step_helper_function_prime(su_mesh, node_tp, face_Optimization_Domain, steepest_descent, step_LowerLimit);
	double helper_function_step_Upper = step_helper_function(su_mesh, node_tp, face_Optimization_Domain, steepest_descent, step_UpperLimit);
	double helper_function_step_Upper_prime = step_helper_function_prime(su_mesh, node_tp, face_Optimization_Domain, steepest_descent, step_UpperLimit);
	// 定义两个变量，作为三次多项式插值的两个辅助值
	double d1 = helper_function_step_Lower_prime + helper_function_step_Upper_prime - 3 * (helper_function_step_Lower - helper_function_step_Upper) / (step_LowerLimit - step_UpperLimit);
	double d2 = data_process.sign(step_UpperLimit - step_LowerLimit) * sqrt(abs(d1 * d1 - helper_function_step_Lower_prime * helper_function_step_Upper_prime));
	// 定义一个变量，储存三次多项式插值法解得的下一次迭代步长值
	double step = step_UpperLimit - (step_UpperLimit - step_LowerLimit) * (helper_function_step_Upper_prime + d2 - d1) / (helper_function_step_Upper_prime - helper_function_step_Lower_prime + 2 * d2);
	return step;
}

double _QUALITY::Zoom(_SU_MESH* su_mesh, NODE node_tp, std::vector<FACE> face_Optimization_Domain, double steepest_descent[3][1], double step_LowerLimit, double step_UpperLimit)
{
	// 首先依据Zoom算法对(x,y)的要求，即ϕ(x)<ϕ(y)，来确定第一次迭代的计算区间
	double step_Low_helper_function;   // 定义两个变量，作为总是满足Zoom函数条件的两个数
	double step_High_helper_function;
	double step_current;            // 声明一个变量，作为当前迭代时的步长
	double helper_function_LowStep;    // 声明一个变量，储存步长为step_Low_helper_function的辅助函数值
	double helper_function_current; // 声明一个变量，储存本次迭代辅助函数值
	double helper_function_primeValue; // 声明一个变量，储存本次迭代辅助函数的导数值
	// 定义一个变量，储存step=0时的辅助函数值
	double helper_function_step0 = step_helper_function(su_mesh, node_tp, face_Optimization_Domain, steepest_descent, 0);
	// 定义一个变量，储存step=0时的辅助函数导数值
	double helper_function_step0_prime = step_helper_function_prime(su_mesh, node_tp, face_Optimization_Domain, steepest_descent, 0);
	double Armijo_right; // 声明一个变量，作为本次迭代Armijo准则的右值
	// 定义一个变量，作为强Wolfe准则的右值，该值为定值，由于牛顿方向是目标函数下降方向，此处导数必然小于0，则取负数作为强Wolfe准则的右值
	double Wolfe_right = -c2 * helper_function_step0_prime;
	if (step_helper_function(su_mesh, node_tp, face_Optimization_Domain, steepest_descent, step_LowerLimit) < step_helper_function(su_mesh, node_tp, face_Optimization_Domain, steepest_descent, step_UpperLimit))
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
	int iter = 0; // 设定一个计数器，避免由于精度误差导致下面的迭代不会终止，以及在总迭代下确实已经达到了最优节点处导致以下迭代不会停止
	while (true)
	{
		// 用三次多项式插值法在区间(step_Low_helper_function,step_High_helper_function)内查找下一次迭代的步长值
		step_current = cubic_interpolation_step(su_mesh, node_tp, face_Optimization_Domain, steepest_descent, step_Low_helper_function, step_High_helper_function);
		// 计算当前步长下辅助函数值
		helper_function_current = step_helper_function(su_mesh, node_tp, face_Optimization_Domain, steepest_descent, step_current);
		Armijo_right = helper_function_step0 + c1 * step_current * helper_function_step0_prime;
		// 计算当前计算区间下步长为step_Low_helper_function的辅助函数值
		helper_function_LowStep = step_helper_function(su_mesh, node_tp, face_Optimization_Domain, steepest_descent, step_Low_helper_function);
		// 用Armijo准则进行判断，若不满足Armijo准则或本次迭代的步长使得辅助函数值大于步长为step_Low_helper_function的辅助函数值时，调整step_High_helper_function的值，进一步缩小计算空间
		if (helper_function_current > Armijo_right || helper_function_current >= helper_function_LowStep)
			step_High_helper_function = step_current;
		else
		{
			// 计算当前步长下辅助函数的导数值
			helper_function_primeValue = step_helper_function_prime(
				su_mesh, node_tp, face_Optimization_Domain, steepest_descent, step_current);
			// 用强Wolfe准则进行判断，若满足强Wolfe准则，说明当前步长是类最优解，直接退出循环
			if (abs(helper_function_primeValue) <= Wolfe_right)
				break;
			// 保证计算区间是连续下降区间
			if (helper_function_primeValue * (step_High_helper_function - step_Low_helper_function) >= 0)
				step_High_helper_function = step_Low_helper_function;
			// 缩小计算区间
			step_Low_helper_function = step_current;
			// 如果当前步长区间的辅助函数值差值很小，直接退出循环
			if (abs(helper_function_current - helper_function_LowStep) < Min_imp)
				break;
		}
		// step太小的时候可以直接返回0值
		if (step_current < Min_step)
			return 0;
		// 避免由于精度误差，导致step=0时就已经是辅助函数的最小值的情况下，该while循环一直进行下去
		if (helper_function_current > helper_function_step0)
			iter++;
		if (iter > 7)
			return 0;
	}
	return step_current;
}

double _QUALITY::Line_Search_Algorithm_with_Wolfe(_SU_MESH* su_mesh, NODE node_tp, std::vector<FACE> face_Optimization_Domain, double steepest_descent[3][1], double Init_step)
{
	double step_before = 0;          // 定义一个变量，作为上一次迭代时的步长，初始化为0
	double step_current = Init_step; // 定义一个变量，作为当前迭代时的步长，初始化为初始步长
	// 定义一个变量，储存当前节点未加任何步长补正下优化域的目标函数值
	double target_function_value_Init = target_Function(su_mesh, node_tp, face_Optimization_Domain);
	// 首先用初始步长来确定初始有效步长
	while (step_helper_function(su_mesh, node_tp, face_Optimization_Domain, steepest_descent, step_current) == -1)
		step_current /= 2;
	step_current /= 2;
	double step_UpperLimit = step_current * 2; // 定义一个变量，作为每次步长迭代的区间上限，初始化为初始有效步长的两倍
	// 定义一个变量，储存上次迭代后辅助函数值（即节点坐标加上步长后的目标函数值），初始化为target_function_value_Init
	double helper_function_before = target_function_value_Init;
	double helper_function_current; // 声明一个变量，储存本次迭代辅助函数值
	double helper_function_primeValue; // 声明一个变量，储存本次迭代辅助函数的导数值
	// 定义一个变量，储存step=0时的辅助函数值
	double helper_function_step0 = step_helper_function(su_mesh, node_tp, face_Optimization_Domain, steepest_descent, 0);
	// 定义一个变量，储存step=0时的辅助函数导数值
	double helper_function_step0_prime = step_helper_function_prime(su_mesh, node_tp, face_Optimization_Domain, steepest_descent, 0);
	double Armijo_right; // 声明一个变量，作为本次迭代Armijo准则的右值
	// 定义一个变量，作为强Wolfe准则的右值，该值为定值，由于牛顿方向是目标函数下降方向，此处导数必然小于0，则取负数作为强Wolfe准则的右值
	double Wolfe_right = -c2 * helper_function_step0_prime;
	int cnt = 0; // 定义一个变量，用来记录当前迭代次数
	// 直接进入循环，以step_current设置为满足强Wolfe条件的步长即类最优解时终止
	// 本循环以查找包含最优解的区间为目标，具体类最优解的查找会调用Zoom()函数
	while (true)
	{
		// 计算当前步长下辅助函数值
		helper_function_current = step_helper_function(su_mesh, node_tp, face_Optimization_Domain, steepest_descent, step_current);
		//if (helper_function_current = step_helper_function(su_mesh, node_tp, face_Optimization_Domain, steepest_descent, step_current) == -1)
		//	return 0;
		Armijo_right = helper_function_step0 + c1 * step_current * helper_function_step0_prime;
		// 用Armijo准则进行判断，若不满足Armijo准则或两次迭代后辅助函数值增加，则说明找到了包含最优解的区间，调用Zoom()函数，退出循环
		if ((helper_function_current > Armijo_right) || (helper_function_current >= helper_function_before && cnt > 1))
		{
			step_current = Zoom(su_mesh, node_tp, face_Optimization_Domain, steepest_descent, step_before, step_current);
			break;
		}
		// 计算当前步长下辅助函数的导数值
		helper_function_primeValue = step_helper_function_prime(su_mesh, node_tp, face_Optimization_Domain, steepest_descent, step_current);
		// 用强Wolfe准则进行判断，若满足强Wolfe准则，说明当前步长是类最优解，直接退出循环
		if (abs(helper_function_primeValue) <= Wolfe_right)
			break;
		// 如果在当前步长下辅助函数的导数值大于等于0，则说明找到了包含最优解的区间，调用Zoom()函数，退出循环
		if (helper_function_primeValue >= 0)
		{
			step_current = Zoom(su_mesh, node_tp, face_Optimization_Domain, steepest_descent, step_current, step_before);
			break;
		}
		// 记录本次迭代步长
		step_before = step_current;
		// 用三次多项式插值法在区间(step_LowerLimit,step_UpperLimit)内查找下一次迭代的步长值
		step_current = cubic_interpolation_step(su_mesh, node_tp, face_Optimization_Domain, steepest_descent, step_current, step_UpperLimit);
		// 记录当前迭代辅助函数值，作为下一次迭代时使用
		helper_function_before = helper_function_current;
		cnt++;
		if (cnt > 13)break;
		if (step_current< step_before || step_current>step_UpperLimit)
			return 0;
	}
	return step_current;
}

void _QUALITY::Construct_Local_Optimization_Domain_All(_SU_MESH* su_mesh, std::vector<int> Init_inferior_elem, std::vector<int>* Laplacian_Smoothing_nodeNum)
{
	// 遍历Init_inferior_elem内所有单元，将非初始网格边界点全部压入Laplacian_Smoothing_nodeNum容器
	for (std::vector<int>::iterator iter = Init_inferior_elem.begin();
		iter != Init_inferior_elem.end();
		++iter)
	{
		// 一个网格单元有四个节点
		for (int i = 0; i < DIM + 1; i++)
		{
			if (su_mesh->elem.at(*iter).form[i] < su_mesh->InitNode_num) // 跳过初始网格边界点
				continue;
			if (std::find(Laplacian_Smoothing_nodeNum->begin(),
				Laplacian_Smoothing_nodeNum->end(),
				su_mesh->elem.at(*iter).form[i]) == Laplacian_Smoothing_nodeNum->end())
				Laplacian_Smoothing_nodeNum->push_back(
					su_mesh->elem.at(*iter).form[i]);
		}
	}
	return;
}

bool _QUALITY::Laplacian_Smoothing(_SU_MESH* su_mesh, int Laplacian_Smoothing_nodeNum)
{
	std::vector<int> elemNum_IncludeNode;          // 声明一个容器，用来储存包含待优化节点的所有网格单元编号
	std::vector<FACE> face_Optimization_Domain;     // 创建一个容器，用来储存局部优化域的边界面
	FACE face_tp;
	double lowest_quality_before = 1; // 定义一个变量，储存节点优化前的局部优化域的最小单元质量，初始化为1
	double lowest_quality_after = 1;  // 定义一个变量，储存节点优化后的局部优化域的最小单元质量，初始化为1
	_MESH_PROCESS mesh_process;
	_DATA_PROCESS data_process;
	// 查找包含待优化节点的所有网格单元编号
	mesh_process.FindBall_fast(su_mesh, Laplacian_Smoothing_nodeNum, &elemNum_IncludeNode);
	// 将所有局部优化域的边界面压入face_Optimization_Domain，并调整面节点顺序，使得以
	// 面点1、面点2、面点3、待优化点的顺序是orient3D规定的正方向
	// 便于后续判断待优化点Laplacian光顺后是否有效
	for (std::vector<int>::iterator iter = elemNum_IncludeNode.begin();
		iter != elemNum_IncludeNode.end();
		++iter)
	{
		switch (mesh_process.ELEM_Include_Node(su_mesh->elem.at(*iter),
			Laplacian_Smoothing_nodeNum))
		{
		case 0:
			face_tp.form[0] = su_mesh->elem.at(*iter).form[1];
			face_tp.form[1] = su_mesh->elem.at(*iter).form[2];
			face_tp.form[2] = su_mesh->elem.at(*iter).form[3];
			break;
		case 1:
			face_tp.form[0] = su_mesh->elem.at(*iter).form[0];
			face_tp.form[1] = su_mesh->elem.at(*iter).form[2];
			face_tp.form[2] = su_mesh->elem.at(*iter).form[3];
			break;
		case 2:
			face_tp.form[0] = su_mesh->elem.at(*iter).form[0];
			face_tp.form[1] = su_mesh->elem.at(*iter).form[1];
			face_tp.form[2] = su_mesh->elem.at(*iter).form[3];
			break;
		case 3:
			face_tp.form[0] = su_mesh->elem.at(*iter).form[0];
			face_tp.form[1] = su_mesh->elem.at(*iter).form[1];
			face_tp.form[2] = su_mesh->elem.at(*iter).form[2];
			break;
		default:
			break;
		}
		// 使得以 面点1、面点2、面点3、待优化点的顺序是orient3D规定的正方向
		if (data_process.tetrahedral_volume(
			su_mesh->node.at(face_tp.form[0]),
			su_mesh->node.at(face_tp.form[1]),
			su_mesh->node.at(face_tp.form[2]),
			su_mesh->node.at(Laplacian_Smoothing_nodeNum)) < 0)
			std::swap(face_tp.form[1], face_tp.form[2]);
		face_Optimization_Domain.push_back(
			face_tp); // 压入face_Optimization_Domain
		lowest_quality_before = su_mesh->elem.at(*iter).quality < lowest_quality_before
			? su_mesh->elem.at(*iter).quality
			: lowest_quality_before;
	}
	// 得到Laplacian光顺后节点位置
	double x_value = 0, x_cnt = 0, y_value = 0, y_cnt = 0, z_value = 0,
		z_cnt = 0;                 // 定义一系列变量，储存x、y与z方向总坐标之和与总点数
	NODE node_tp = su_mesh->node.at(
		Laplacian_Smoothing_nodeNum); // 定义一个变量，储存优化后节点，初始化为待优化点，便于储存待优化点的密度等信息
	for (std::vector<FACE>::iterator iter = face_Optimization_Domain.begin();
		iter != face_Optimization_Domain.end();
		++iter)
		// 一个局部优化域的边界面有三个节点
		for (int i = 0; i < DIM; i++)
		{
			x_value += su_mesh->node.at(iter->form[i]).pos[0];
			x_cnt++;
			y_value += su_mesh->node.at(iter->form[i]).pos[1];
			y_cnt++;
			z_value += su_mesh->node.at(iter->form[i]).pos[2];
			z_cnt++;
		}
	node_tp.pos[0] = x_value / x_cnt; // Laplacian光顺后节点x坐标
	node_tp.pos[1] = y_value / y_cnt; // Laplacian光顺后节点y坐标
	node_tp.pos[2] = z_value / z_cnt; // Laplacian光顺后节点z坐标
	// 计算优化后节点与各局部优化域的边界面形成的四面体质量，若有小于0的值，说明该优化点位置无效，直接返回false，并且记录优化后的局部优化域的最小单元质量
	double quality_tp; // 定义一个变量，储存临时单元质量
	for (std::vector<FACE>::iterator iter = face_Optimization_Domain.begin();
		iter != face_Optimization_Domain.end();
		++iter)
	{
		quality_tp = Shape_Quality(su_mesh->node.at(iter->form[0]),
			su_mesh->node.at(iter->form[1]),
			su_mesh->node.at(iter->form[2]),
			node_tp);
		// 如果当前四面体质量小于0，说明该优化点位置无效，直接返回false
		if (quality_tp < 0)
			return false;
		// 记录优化后的局部优化域的最小单元质量
		lowest_quality_after = quality_tp < lowest_quality_after
			? quality_tp
			: lowest_quality_after;
		// 每次迭代都动态判断一下当前最小单元质量与优化前最小单元质量，若小于则直接返回false
		if (lowest_quality_after < lowest_quality_before)
			return false;
	}
	// 如果优化后的局部优化域的最小单元质量大于优化前的局部优化域的最小单元质量，则确定优化成功，修改待优化点坐标，并且修改局部优化域内每一个网格单元的单元质量
	if (lowest_quality_after > lowest_quality_before)
	{
		su_mesh->node.at(Laplacian_Smoothing_nodeNum) = node_tp;
		// 修改局部优化域内每一个网格单元的单元质量
		for (std::vector<int>::iterator iter = elemNum_IncludeNode.begin();
			iter != elemNum_IncludeNode.end();
			++iter)
			su_mesh->elem.at(*iter).quality = abs(Shape_Quality(
				su_mesh->node.at(su_mesh->elem.at(*iter).form[0]),
				su_mesh->node.at(su_mesh->elem.at(*iter).form[1]),
				su_mesh->node.at(su_mesh->elem.at(*iter).form[2]),
				su_mesh->node.at(su_mesh->elem.at(*iter).form[3])));
		return true;
	}
	else
		return false;
}

bool _QUALITY::Optimization_based_Smoothing_The_all(_SU_MESH* su_mesh, int Optimization_based_Smoothing_nodeNum)
{
	double target_function_value_before = 0; // 定义一个变量，储存上次迭代后目标函数值，初始化为0
	double target_function_value_after = 0;  // 定义一个变量，储存本次迭代后目标函数值，初始化为0
	// *首先得到局部优化域的初始目标函数值
	std::vector<int> elemNum_IncludeNode; // 声明一个容器，用来储存包含待优化节点的所有网格单元编号
	_MESH_PROCESS mesh_process;
	_DATA_PROCESS data_process;
	// 查找包含待优化节点的所有网格单元编号
	mesh_process.FindBall_fast(su_mesh, Optimization_based_Smoothing_nodeNum, &elemNum_IncludeNode);
	// 目标函数为该节点相邻单元的所有错误函数相加，此处直接累加所有单元的质量倒数即可
	for (std::vector<int>::iterator iter = elemNum_IncludeNode.begin(); iter != elemNum_IncludeNode.end(); ++iter)
		target_function_value_before += 1 / su_mesh->elem.at(*iter).quality;
	// *储存局部优化域的边界面
	std::vector<FACE> face_Optimization_Domain; // 创建一个容器，用来储存局部优化域的边界面
	FACE face_tp;
	// 将所有局部优化域的边界面压入face_Optimization_Domain，并调整边节点顺序，使得以
	// 面点1、面点2、面点3、待优化点的顺序是orient3D规定的正方向
	// 便于后续判断待优化点Optimization_based光顺后是否有效
	for (std::vector<int>::iterator iter = elemNum_IncludeNode.begin(); iter != elemNum_IncludeNode.end(); ++iter)
	{
		switch (mesh_process.ELEM_Include_Node(su_mesh->elem.at(*iter), Optimization_based_Smoothing_nodeNum))
		{
		case 0:
			face_tp.form[0] = su_mesh->elem.at(*iter).form[1];
			face_tp.form[1] = su_mesh->elem.at(*iter).form[2];
			face_tp.form[2] = su_mesh->elem.at(*iter).form[3];
			break;
		case 1:
			face_tp.form[0] = su_mesh->elem.at(*iter).form[0];
			face_tp.form[1] = su_mesh->elem.at(*iter).form[2];
			face_tp.form[2] = su_mesh->elem.at(*iter).form[3];
			break;
		case 2:
			face_tp.form[0] = su_mesh->elem.at(*iter).form[0];
			face_tp.form[1] = su_mesh->elem.at(*iter).form[1];
			face_tp.form[2] = su_mesh->elem.at(*iter).form[3];
			break;
		case 3:
			face_tp.form[0] = su_mesh->elem.at(*iter).form[0];
			face_tp.form[1] = su_mesh->elem.at(*iter).form[1];
			face_tp.form[2] = su_mesh->elem.at(*iter).form[2];
			break;
		default:
			break;
		}
		// 使得以 面点1、面点2、面点3、待优化点的顺序是orient3D规定的正方向
		if (data_process.tetrahedral_volume(su_mesh->node.at(face_tp.form[0]),
			su_mesh->node.at(face_tp.form[1]),
			su_mesh->node.at(face_tp.form[2]),
			su_mesh->node.at(Optimization_based_Smoothing_nodeNum)) < 0)
			std::swap(face_tp.form[1], face_tp.form[2]);
		face_Optimization_Domain.push_back(face_tp);                               // 压入face_Optimization_Domain
	}
	NODE node_original = su_mesh->node.at(Optimization_based_Smoothing_nodeNum); // 定义一个NODE类，作为原始点，即程序输入的待优化点
	NODE node_before = node_original;                                            // 声明一个NODE类，储存本次迭代前点位置，初始化为原始点位置
	NODE node_after;                                                             // 声明一个NODE类，储存本次迭代后点位置
	NODE node_Delta;                                                             // 声明一个NODE类，作为每次迭代的节点坐标变化值
	double gradient_vector[3][1];                                                // 声明一个变量，作为梯度向量，也即当前位置目标函数最快下降方向
	double hessian_matrix[3][3];                                                 // 声明一个变量，作为hessian矩阵
	// 声明一个变量，作为当前迭代位置的最速下降方向，利用牛顿法取修正的hessian逆矩阵与梯度向量乘积的负数作为牛顿方向即最速下降方向
	double steepest_descent[3][1];
	double steepest_descent_Module; // 最速下降方向向量的模
	double optimal_step = 0;        // 定义一个变量，作为每次迭代的类最优步长，并且下次迭代的初始步长取为该值，初始化为0
	int iter = 0;                   // 声明一个变量，作为迭代次数
	// * 利用带线搜索的修正牛顿法来求每次迭代的节点坐标变化
	do
	{
		// 储存上次迭代结束后目标函数值，第一次迭代时，不需要储存
		if (target_function_value_after != 0) target_function_value_before = target_function_value_after;
		// 先由当前点位置求目标函数当前位置梯度
		for (int i = 0; i < 3; i++)
			if ((gradient_vector[i][0] = target_Function_first_derivative(su_mesh, node_before, face_Optimization_Domain, i)) == -1)
			{
				iter++;
				break;
				std::cout << "target_Function_first_derivative value =-1!\n";
				exit(-1);
			}
		// 梯度向量的模小于最小梯度值时代表达到极值点处，停止迭代
		if (data_process.Vector_Module(gradient_vector) < Min_gradient)
			break;
		// 求当前点位置目标函数的hessian矩阵
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				if ((hessian_matrix[i][j] = target_Function_second_derivative(su_mesh, node_before, face_Optimization_Domain, i, j)) == -1)
				{
					iter++;
					break;
					std::cout << "target_Function_second_derivative value =-1!\n";
					exit(-1);
				}
		// 判断hessian矩阵是否正定，若不正定则修改其值使其正定
		data_process.Positivity_Hessian_Matrix(hessian_matrix);
		// 对hessian矩阵矩阵求逆
		data_process.Inverse_Matrix(hessian_matrix);
		// 求牛顿方向即当前位置最速下降方向，即负的hessian逆矩阵乘梯度
		data_process.matrix_product(hessian_matrix, gradient_vector, steepest_descent);
		steepest_descent[0][0] = -steepest_descent[0][0];
		steepest_descent[1][0] = -steepest_descent[1][0];
		steepest_descent[2][0] = -steepest_descent[2][0];
		// 求最速下降方向向量的模
		steepest_descent_Module = data_process.Vector_Module(steepest_descent);
		// 最速下降方向向量的模小于最小最速下降方向模时代表达到极值点处，停止迭代
		if (steepest_descent_Module < Min_steepest_descent)
			break;
		// 为避免不必要的误差，每一次迭代时，需要根据最速下降方向梯度的模来自适应地确定第一次迭代的初始步长
		// 设定为在当前最速下降方向梯度下，待优化节点最大移动长度为理想的最大边边长
		optimal_step = su_mesh->longest_distance / steepest_descent_Module;
		// 采用强Wolfe准则，利用牛顿方向，利用上次迭代的最优步长的两倍作为本次迭代的初始步长
		optimal_step = Line_Search_Algorithm_with_Wolfe(su_mesh, node_before, face_Optimization_Domain, steepest_descent, optimal_step);
		if (optimal_step == 0)
			break;
		// 求本次迭代节点坐标变化
		node_Delta = get_Node_Delta(optimal_step, steepest_descent);
		node_after = node_before + node_Delta;
		target_function_value_after = target_Function(su_mesh, node_after, face_Optimization_Domain);
		if (target_function_value_after == -1)
		{
			std::cout << "target_function_value_after=-1\n";
			exit(-1);
		}
		node_before = node_after;
		iter++;
	} while (optimal_step > Min_step && iter < Max_iter && abs(target_function_value_after - target_function_value_before) > Min_imp);
	su_mesh->node.at(Optimization_based_Smoothing_nodeNum) = node_before;
	// 修改局部优化域内每一个网格单元的单元质量
	for (std::vector<int>::iterator iter = elemNum_IncludeNode.begin(); iter != elemNum_IncludeNode.end(); ++iter)
		su_mesh->elem.at(*iter).quality = abs(Shape_Quality(su_mesh->node.at(su_mesh->elem.at(*iter).form[0]),
			su_mesh->node.at(su_mesh->elem.at(*iter).form[1]),
			su_mesh->node.at(su_mesh->elem.at(*iter).form[2]),
			su_mesh->node.at(su_mesh->elem.at(*iter).form[3])));
	return true;
}

void _QUALITY::Quality_Information(_SU_MESH* su_mesh)
{
	int low_quality_elem = 0;  // 定义一个变量，储存劣质网格数量
	double all_quality = 0;    // 定义一个变量，储存所有网格单元的质量，初始化为0
	double lowest_quality = 1; // 定义一个变量，储存网格单元的最小质量，初始化为1
	std::vector<double> Shape_Quality;
	// 搜索各网格单元质量
	for (std::vector<ELEM>::iterator elem_iter = su_mesh->elem.begin(); elem_iter != su_mesh->elem.end(); ++elem_iter)
	{
		Shape_Quality.push_back(elem_iter->quality);
		if (elem_iter->quality < Minimum_Shape_Quality_Smoothing)
			low_quality_elem++;
		all_quality += elem_iter->quality;
		if (elem_iter->quality < lowest_quality)
			lowest_quality = elem_iter->quality;
	}
	std::sort(Shape_Quality.begin(), Shape_Quality.end());
	int quality_cnt[11] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	double quality_level = 0.02;
	int cnt = 0;
	for (std::vector<double>::iterator iter = Shape_Quality.begin(); iter != Shape_Quality.end(); )
	{
		if (*iter <= quality_level)
			quality_cnt[cnt] = quality_cnt[cnt] + 1;
		else
		{
			cnt++;
			if (quality_level == 0.02)
				quality_level = 0.1;
			else
				quality_level = quality_level + 0.1;
			if (iter != Shape_Quality.begin())
				--iter;
			else
				continue;
		}
		++iter;
	}
	std::cout << "The average quality of the current triangulation is "
		<< all_quality / su_mesh->elem_num << ".\n";
	std::cout << "The lowest quality of the current triangulation is "
		<< lowest_quality << ".\n";
	std::cout << "The number of Bad Meshes is " << low_quality_elem << std::endl;
	int number = 0;
	for (int i = 0; i < 11; i++)
	{
		number += quality_cnt[i];
		std::cout << quality_cnt[i] << ' ';
	}
	if (number == su_mesh->elem_num)
		std::cout << su_mesh->elem_num << '\n';
	else
		std::cout << "elem_num error!\n";
	return;
}

void _QUALITY::Quality_Optimization_SliverRemoval(_SU_MESH* su_mesh)
{
	std::vector<int> Sliver_elem; // 声明一个容器，储存广义薄元单元编号
	// 遍历elem容器，储存广义薄元单元编号，网格质量小于Minimum_Shape_Quality_SliverRemoval的都被认定为广义薄元
	for (int i = 0; i < su_mesh->elem_num; i++)
		if (su_mesh->elem.at(i).quality < Minimum_Shape_Quality_SliverRemoval)
			Sliver_elem.push_back(i);
	std::string Sliver_type;     // 储存广义薄元类型，分别是Cap、Sliver、Spade、Wedge四个单元类型
	bool Decompose_Sliver_judge; // 判断当前广义薄元是否分解成功
	// 遍历Sliver_elem容器，判断每个广义薄元类型，对每种类型的广义薄元，分别使用不同的方法去分解
	for (std::vector<int>::iterator elemNum_iter = Sliver_elem.begin(); elemNum_iter != Sliver_elem.end(); ++elemNum_iter)
	{
		Sliver_type = Check_Sliver_type(su_mesh->node.at(su_mesh->elem.at(*elemNum_iter).form[0]),
			su_mesh->node.at(su_mesh->elem.at(*elemNum_iter).form[1]),
			su_mesh->node.at(su_mesh->elem.at(*elemNum_iter).form[2]),
			su_mesh->node.at(su_mesh->elem.at(*elemNum_iter).form[3]));
		// 判断当前广义薄元类型，使用不同方法分解，成功分解时，程序内部会自动更新被修改过的网格单元各种信息
		if (Sliver_type == "Cap")
			Decompose_Sliver_judge = Decompose_Cap(su_mesh, *elemNum_iter);
		else if (Sliver_type == "Sliver")
			Decompose_Sliver_judge = Decompose_Sliver(su_mesh, *elemNum_iter);
		else if (Sliver_type == "Spade")
			Decompose_Sliver_judge = Decompose_Spade(su_mesh, *elemNum_iter);
		else if (Sliver_type == "Wedge")
			Decompose_Sliver_judge = Decompose_Wedge(su_mesh, *elemNum_iter);
		else
		{
			std::cout << "Generalized thin element judgment error, please "
				"check the program!\n";
			exit(-1);
		}
		if (Decompose_Sliver_judge == false)
		{
			std::cout << "Generalized thin element decomposition failed!\n";
			exit(-1);
		}
	}
	return;
}

void _QUALITY::Quality_Optimization_Smoothing(_SU_MESH* su_mesh)
{
	std::vector<int> Init_inferior_elem; // 声明一个容器，储存初始劣质单元编号
	// 遍历elem容器，储存初始劣质单元编号
	for (int i = 0; i < su_mesh->elem_num; i++)
		if (su_mesh->elem.at(i).quality < Minimum_Shape_Quality_Smoothing)
			Init_inferior_elem.push_back(i);
	// 构造局部优化域，用待优化节点编号来代表每个优化域，具体优化域则是当前三角化中包含节点的所有网格单元
	std::vector<int> Laplacian_Smoothing_nodeNum; // 声明一个容器，储存需要进行Laplacian光顺的节点编号
	Construct_Local_Optimization_Domain_All(su_mesh, Init_inferior_elem, &Laplacian_Smoothing_nodeNum);
	// 进行智能Laplacian光顺对每一个节点的位置进行调整
	//for (std::vector<int>::iterator iter = Laplacian_Smoothing_nodeNum.begin(); iter != Laplacian_Smoothing_nodeNum.end(); ++iter)
	//	Laplacian_Smoothing(su_mesh, *iter);
	// 遍历Laplacian_Smoothing_nodeNum容器（即优化后的所有节点），判断每个节点的ball内是否全是高质量单元
	// 若依然包含劣质单元，则插入Optimization_based_Smoothing_nodeNum容器，进行Optimization_based光顺
	std::vector<int> Optimization_based_Smoothing_nodeNum; // 声明一个容器，储存需要进行Optimization_based光顺的节点编号
	std::vector<int> elemNum_IncludeNode;                  // 声明一个容器，用来储存包含待判断节点的所有网格单元编号
	_MESH_PROCESS mesh_process;
	for (std::vector<int>::iterator nodeNum_iter = Laplacian_Smoothing_nodeNum.begin(); nodeNum_iter != Laplacian_Smoothing_nodeNum.end(); ++nodeNum_iter)
	{
		std::vector<int>().swap(elemNum_IncludeNode); // 初始化elemNum_IncludeNode，并释放容器空间
		mesh_process.FindBall_fast(su_mesh, *nodeNum_iter, &elemNum_IncludeNode);
		// 遍历elemNum_IncludeNode，若包含劣质网格则将当前判断节点插入Optimization_based_Smoothing_nodeNum容器
		for (std::vector<int>::iterator elemNum_iter = elemNum_IncludeNode.begin(); elemNum_iter != elemNum_IncludeNode.end(); ++elemNum_iter)
			if (su_mesh->elem.at(*elemNum_iter).quality < Minimum_Shape_Quality_Smoothing)
			{
				Optimization_based_Smoothing_nodeNum.push_back(*nodeNum_iter);
				break;
			}
	}
	// 进行Optimization_based光顺对每一个节点的位置进行调整
	for (std::vector<int>::iterator iter = Optimization_based_Smoothing_nodeNum.begin(); iter != Optimization_based_Smoothing_nodeNum.end(); ++iter)
		Optimization_based_Smoothing_The_all(su_mesh, *iter);
	//Optimization_based_Smoothing_Compared(su_mesh, *iter);
	return;
}

double _QUALITY::Face_Transform_23_Legality(_SU_MESH* su_mesh, FACE face_tp, std::vector<int> elemNum_include_face)
{
	// 面交换的合法性判断其实就是比较交换前后的网格单元总体积，若其差值大于设定阈值，则说明网格当前交换不合法，返回-1
	ELEM elem_tp[2];
	double all_volume_before = 0;    // 储存面交换前网格单元总体积
	double all_volume_after = 0;     // 储存面交换后网格单元总体积
	double min_quality_before = 1.0; // 储存面交换前交换域内最小的网格单元质量
	double min_quality_after = 1.0;  // 储存面交换后交换域内最小的网格单元质量
	std::vector<int> nodeNum;        // 声明一个容器，储存交换域的所有网格节点编号
	_DATA_PROCESS data_process;
	// 取出elemNum_include_face容器内所有网格单元
	for (int i = 0; i < 2; i++)
	{
		elem_tp[i] = su_mesh->elem.at(elemNum_include_face.at(i));
		all_volume_before += abs(data_process.tetrahedral_volume(
			su_mesh->node.at(elem_tp[i].form[0]),
			su_mesh->node.at(elem_tp[i].form[1]),
			su_mesh->node.at(elem_tp[i].form[2]),
			su_mesh->node.at(elem_tp[i].form[3])));
		for (int j = 0; j < DIM + 1; j++)
			if (std::find(nodeNum.begin(), nodeNum.end(), elem_tp[i].form[j]) == nodeNum.end())
				nodeNum.push_back(elem_tp[i].form[j]);
		if (min_quality_before > elem_tp[i].quality)
			min_quality_before = elem_tp[i].quality;
	}
	EDGE edge_tp; // 声明一个EDGE类，储存进行面交换后，三个网格单元的共用边
	int cnt = 0;
	for (std::vector<int>::iterator iter = nodeNum.begin();
		iter != nodeNum.end();
		++iter)
		if (*iter != face_tp.form[0] && *iter != face_tp.form[1] && *iter != face_tp.form[2])
			edge_tp.form[cnt++] = *iter;
	edge_tp.Sort();
	for (int i = 0; i < 3; i++)
		for (int j = i + 1; j < 3; j++)
			all_volume_after += abs(data_process.tetrahedral_volume(
				su_mesh->node.at(edge_tp.form[0]),
				su_mesh->node.at(edge_tp.form[1]),
				su_mesh->node.at(face_tp.form[i]),
				su_mesh->node.at(face_tp.form[j])));
	// 若其差值大于设定阈值，则说明网格当前交换不合法，返回-1
	if (abs(all_volume_after - all_volume_before) > Delta_volume)
		return -1;
	// 储存进行面交换后形成的局部交换域内的最低网格质量
	double quality_tp;
	for (int i = 0; i < 3; i++)
		for (int j = i + 1; j < 3; j++)
		{
			quality_tp = abs(Shape_Quality(su_mesh->node.at(edge_tp.form[0]),
				su_mesh->node.at(edge_tp.form[1]),
				su_mesh->node.at(face_tp.form[i]),
				su_mesh->node.at(face_tp.form[j])));
			if (min_quality_after > quality_tp)
				min_quality_after = quality_tp;
		}
	if (min_quality_after >= min_quality_before)
		return min_quality_after;
	else
		return -1;
}

std::vector<int> _QUALITY::Face_Transform_23(_SU_MESH* su_mesh, FACE face_tp)
{
	std::vector<int> elemNum_include_face;
	_MESH_PROCESS mesh_process;
	mesh_process.FindAwl(su_mesh, face_tp, &elemNum_include_face, "fast");
	ELEM elem_tp[2];
	// 取出elemNum_include_face容器内所有网格单元
	for (int i = 0; i < 2; i++)
		elem_tp[i] = su_mesh->elem.at(elemNum_include_face.at(i));
	// 储存T23交换后交换域内三个网格单元的共同相邻边
	EDGE edge_adjacent(elem_tp[0].form[mesh_process.Face_Opposite_Node(elem_tp[0], face_tp)], elem_tp[1].form[mesh_process.Face_Opposite_Node(elem_tp[1], face_tp)]);
	edge_adjacent.Sort();
	// 储存新生成的网格单元，默认第一个网格单元使用elemNum_include_edge的第一个网格单元编号，以此类推，新生成的网格单元编号使用su_mesh->elem_num
	// 将su_mesh->elem_num压入elemNum_include_face，便于后续使用
	elemNum_include_face.push_back(su_mesh->elem_num);
	std::vector<int> elemNum_modify; // 声明一个int类型容器，储存成功面交换后网格节点有过变动的所有网格单元编号
	elemNum_modify.push_back(elemNum_include_face.at(0));
	elemNum_modify.push_back(elemNum_include_face.at(1));
	elemNum_modify.push_back(elemNum_include_face.at(2));
	ELEM elem_new[3] = { {edge_adjacent.form[0], edge_adjacent.form[1], face_tp.form[0], face_tp.form[1], -1, -1, elemNum_include_face.at(1), elemNum_include_face.at(2)},
					   {edge_adjacent.form[0], edge_adjacent.form[1], face_tp.form[1], face_tp.form[2], -1, -1, elemNum_include_face.at(2), elemNum_include_face.at(0)},
					   {edge_adjacent.form[0], edge_adjacent.form[1], face_tp.form[0], face_tp.form[2], -1, -1, elemNum_include_face.at(1), elemNum_include_face.at(0)} };
	// 为加速面交换后网格相邻信息的更新，首先储存面交换前交换域的边界面以及交换域外包含边界面的网格单元编号
	// 并且只需要储存交换域内网格单元在face_tp三个节点的相对网格面
	std::vector<int> elemNum_adjacent;
	int elemNum_iter;
	std::vector<FACE> face_adjacent;
	std::vector<FACE>::iterator face_iter;
	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 3; j++)
		{
			elemNum_adjacent.push_back(elem_tp[i].neig[mesh_process.ELEM_Include_Node(elem_tp[i], face_tp.form[j])]);
			face_adjacent.push_back(mesh_process.Node_Opposite_Face(elem_tp[i], face_tp.form[j]));
		}
	FACE face_judge[] = { {edge_adjacent.form[0], face_tp.form[0], face_tp.form[1]},
					  {edge_adjacent.form[0], face_tp.form[0], face_tp.form[2]},
					  {edge_adjacent.form[0], face_tp.form[1], face_tp.form[2]},
					  {edge_adjacent.form[1], face_tp.form[0], face_tp.form[1]},
					  {edge_adjacent.form[1], face_tp.form[0], face_tp.form[2]},
					  {edge_adjacent.form[1], face_tp.form[1], face_tp.form[2]} };
	for (int i = 0; i < 6; i++)
		face_judge[i].Sort();
	elem_new[0].neig[0] = elemNum_adjacent.at(elemNum_iter = std::distance(face_adjacent.begin(), face_iter = std::find(face_adjacent.begin(), face_adjacent.end(), face_judge[3])));
	elem_new[0].neig[1] = elemNum_adjacent.at(elemNum_iter = std::distance(face_adjacent.begin(), face_iter = std::find(face_adjacent.begin(), face_adjacent.end(), face_judge[0])));
	elem_new[1].neig[0] = elemNum_adjacent.at(elemNum_iter = std::distance(face_adjacent.begin(), face_iter = std::find(face_adjacent.begin(), face_adjacent.end(), face_judge[5])));
	elem_new[1].neig[1] = elemNum_adjacent.at(elemNum_iter = std::distance(face_adjacent.begin(), face_iter = std::find(face_adjacent.begin(), face_adjacent.end(), face_judge[2])));
	elem_new[2].neig[0] = elemNum_adjacent.at(elemNum_iter = std::distance(face_adjacent.begin(), face_iter = std::find(face_adjacent.begin(), face_adjacent.end(), face_judge[4])));
	elem_new[2].neig[1] = elemNum_adjacent.at(elemNum_iter = std::distance(face_adjacent.begin(), face_iter = std::find(face_adjacent.begin(), face_adjacent.end(), face_judge[1])));
	for (int i = 0; i < 3; i++)
		elem_new[i].Sort();
	// 替换elem容器内值，并将新生成的网格单元压入elem容器
	su_mesh->elem.at(elemNum_include_face.at(0)) = elem_new[0];
	su_mesh->elem.at(elemNum_include_face.at(1)) = elem_new[1];
	su_mesh->elem.push_back(elem_new[2]);
	su_mesh->elem_num = su_mesh->elem_num + 1;
	int value_tp;
	// 修改所有相邻信息
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < DIM + 1; j++)
		{
			if (elem_new[i].neig[j] == -1)
				continue;
			value_tp = mesh_process.Face_Opposite_Node(su_mesh->elem.at(elem_new[i].neig[j]),
				mesh_process.Node_Opposite_Face(elem_new[i], elem_new[i].form[j]));
			if (value_tp == -1)
			{
				std::cout << "Face_Transform_23 run error, please check the program!\n";
				exit(-1);
			}
			su_mesh->elem.at(elem_new[i].neig[j]).neig[value_tp] = elemNum_include_face.at(i);
		}
		// 修改单元所有节点的elem值
		mesh_process.Renew_NodeElem(su_mesh, elemNum_include_face.at(i));
		// 修改单元质量
		su_mesh->elem.at(elemNum_include_face.at(i)).quality =
			abs(Shape_Quality(su_mesh->node.at(su_mesh->elem.at(elemNum_include_face.at(i)).form[0]),
				su_mesh->node.at(su_mesh->elem.at(elemNum_include_face.at(i)).form[1]),
				su_mesh->node.at(su_mesh->elem.at(elemNum_include_face.at(i)).form[2]),
				su_mesh->node.at(su_mesh->elem.at(elemNum_include_face.at(i)).form[3])));
	}
	return elemNum_modify;
}

double _QUALITY::Face_Transform_32_Legality(_SU_MESH* su_mesh, EDGE edge_tp, std::vector<int> elemNum_include_edge)
{
	// 面交换的合法性判断其实就是比较交换前后的网格单元总体积，若其差值大于设定阈值，则说明网格当前交换不合法，返回-1
	ELEM elem_tp[3];
	double all_volume_before = 0;    // 储存面交换前网格单元总体积
	double all_volume_after = 0;     // 储存面交换后网格单元总体积
	double min_quality_before = 1.0; // 储存面交换前交换域内最小的网格单元质量
	double min_quality_after = 1.0;  // 储存面交换后交换域内最小的网格单元质量
	std::vector<int> nodeNum;        // 声明一个容器，储存交换域的所有网格节点编号
	_DATA_PROCESS data_process;
	// 取出elemNum_include_edge容器内所有网格单元
	for (int i = 0; i < 3; i++)
	{
		elem_tp[i] = su_mesh->elem.at(elemNum_include_edge.at(i));
		all_volume_before += abs(data_process.tetrahedral_volume(
			su_mesh->node.at(elem_tp[i].form[0]),
			su_mesh->node.at(elem_tp[i].form[1]),
			su_mesh->node.at(elem_tp[i].form[2]),
			su_mesh->node.at(elem_tp[i].form[3])));
		for (int j = 0; j < DIM + 1; j++)
			if (std::find(nodeNum.begin(), nodeNum.end(), elem_tp[i].form[j]) == nodeNum.end())
				nodeNum.push_back(elem_tp[i].form[j]);
		if (min_quality_before > elem_tp[i].quality)
			min_quality_before = elem_tp[i].quality;
	}
	FACE face_tp; // 声明一个FACE类，储存进行面交换后，两个网格单元的相对面
	int cnt = 0;
	for (std::vector<int>::iterator iter = nodeNum.begin();
		iter != nodeNum.end();
		++iter)
		if (*iter != edge_tp.form[0] && *iter != edge_tp.form[1])
			face_tp.form[cnt++] = *iter;
	face_tp.Sort();
	for (int i = 0; i < 2; i++)
		all_volume_after += abs(data_process.tetrahedral_volume(
			su_mesh->node.at(face_tp.form[0]), su_mesh->node.at(face_tp.form[1]), su_mesh->node.at(face_tp.form[2]), su_mesh->node.at(edge_tp.form[i])));
	// 若其差值大于设定阈值，则说明网格当前交换不合法，返回-1
	if (abs(all_volume_after - all_volume_before) > Delta_volume)
		return -1;
	// 储存进行面交换后形成的局部交换域内的最低网格质量
	double quality_tp;
	for (int i = 0; i < 2; i++)
	{
		quality_tp = abs(Shape_Quality(su_mesh->node.at(face_tp.form[0]),
			su_mesh->node.at(face_tp.form[1]),
			su_mesh->node.at(face_tp.form[2]),
			su_mesh->node.at(edge_tp.form[i])));
		if (min_quality_after > quality_tp)
			min_quality_after = quality_tp;
	}
	if (min_quality_after >= min_quality_before)
		return min_quality_after;
	else
		return -1;
}

std::vector<int> _QUALITY::Face_Transform_32(_SU_MESH* su_mesh, EDGE edge_tp)
{
	std::vector<int> elemNum_include_edge;
	_MESH_PROCESS mesh_process;
	mesh_process.FindRing(su_mesh, edge_tp, &elemNum_include_edge, "fast");
	std::sort(elemNum_include_edge.begin(), elemNum_include_edge.end());
	ELEM elem_tp[3];
	std::vector<int> nodeNum; // 声明一个容器，储存交换域的所有网格节点编号
	// 取出elemNum_include_edge容器内所有网格单元
	for (int i = 0; i < 3; i++)
	{
		elem_tp[i] = su_mesh->elem.at(elemNum_include_edge.at(i));
		for (int j = 0; j < DIM + 1; j++)
			if (std::find(nodeNum.begin(), nodeNum.end(), elem_tp[i].form[j]) == nodeNum.end())
				nodeNum.push_back(elem_tp[i].form[j]);
	}
	FACE face_opposite; // 声明一个FACE类，储存进行面交换后，两个网格单元的相对面
	int cnt = 0;
	for (std::vector<int>::iterator iter = nodeNum.begin(); iter != nodeNum.end(); ++iter)
		if (*iter != edge_tp.form[0] && *iter != edge_tp.form[1])
			face_opposite.form[cnt++] = *iter;
	face_opposite.Sort();
	// 储存新生成的网格单元，默认第一个网格单元使用elemNum_include_edge的第一个网格单元编号，以此类推
	std::vector<int> elemNum_modify; // 声明一个int类型容器，储存成功面交换后网格节点有过变动的所有网格单元编号
	elemNum_modify.push_back(elemNum_include_edge.at(0));
	elemNum_modify.push_back(elemNum_include_edge.at(1));
	ELEM elem_new[2] = { {edge_tp.form[0], face_opposite.form[0], face_opposite.form[1], face_opposite.form[2], elemNum_include_edge.at(1), -1, -1, -1},
						{edge_tp.form[1], face_opposite.form[0], face_opposite.form[1], face_opposite.form[2], elemNum_include_edge.at(0), -1, -1, -1} };
	// 为加速面交换后网格相邻信息的更新，首先储存面交换前交换域的边界面以及交换域外包含边界面的网格单元编号
	// 并且只需要储存交换域内网格单元在edge_tp两个节点的相对网格面
	std::vector<int> elemNum_adjacent;
	int elemNum_iter;
	std::vector<FACE> face_adjacent;
	std::vector<FACE>::iterator face_iter;
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 2; j++)
		{
			elemNum_adjacent.push_back(elem_tp[i].neig[mesh_process.ELEM_Include_Node(elem_tp[i], edge_tp.form[j])]);
			face_adjacent.push_back(mesh_process.Node_Opposite_Face(elem_tp[i], edge_tp.form[j]));
		}
	FACE face_tp[] = { {edge_tp.form[0], face_opposite.form[1], face_opposite.form[2]},
					  {edge_tp.form[0], face_opposite.form[0], face_opposite.form[2]},
					  {edge_tp.form[0], face_opposite.form[0], face_opposite.form[1]},
					  {edge_tp.form[1], face_opposite.form[1], face_opposite.form[2]},
					  {edge_tp.form[1], face_opposite.form[0], face_opposite.form[2]},
					  {edge_tp.form[1], face_opposite.form[0], face_opposite.form[1]} };
	for (int i = 0; i < 6; i++)
		face_tp[i].Sort();
	elem_new[0].neig[1] = elemNum_adjacent.at(elemNum_iter = std::distance(face_adjacent.begin(),
		face_iter = std::find(face_adjacent.begin(),
			face_adjacent.end(),
			face_tp[0])));
	elem_new[0].neig[2] = elemNum_adjacent.at(elemNum_iter = std::distance(face_adjacent.begin(),
		face_iter = std::find(face_adjacent.begin(),
			face_adjacent.end(),
			face_tp[1])));
	elem_new[0].neig[3] = elemNum_adjacent.at(elemNum_iter = std::distance(face_adjacent.begin(),
		face_iter = std::find(face_adjacent.begin(),
			face_adjacent.end(),
			face_tp[2])));
	elem_new[1].neig[1] = elemNum_adjacent.at(elemNum_iter = std::distance(face_adjacent.begin(),
		face_iter = std::find(face_adjacent.begin(),
			face_adjacent.end(),
			face_tp[3])));
	elem_new[1].neig[2] = elemNum_adjacent.at(elemNum_iter = std::distance(face_adjacent.begin(),
		face_iter = std::find(face_adjacent.begin(),
			face_adjacent.end(),
			face_tp[4])));
	elem_new[1].neig[3] = elemNum_adjacent.at(elemNum_iter = std::distance(face_adjacent.begin(),
		face_iter = std::find(face_adjacent.begin(),
			face_adjacent.end(),
			face_tp[5])));
	elem_new[0].Sort();
	elem_new[1].Sort();
	// 替换elem容器内值前，将另一个未被使用的网格单元编号与容器内最后一个元素交换位置，然后在容器内删除该元素，实现该网格单元的删除
	_BOUNDARY_RECOVERY boundary_recovery;
	if (elemNum_include_edge.at(2) != (su_mesh->elem_num - 1))
		boundary_recovery.ReplaceElem_two(su_mesh, elemNum_include_edge.at(2), su_mesh->elem_num - 1);
	boundary_recovery.Removal_LastElem(su_mesh, su_mesh->elem_num - 1);
	su_mesh->elem.at(elemNum_include_edge.at(0)) = elem_new[0];
	su_mesh->elem.at(elemNum_include_edge.at(1)) = elem_new[1];
	int value_tp;
	// 修改所有相邻信息
	for (int i = 0; i < 2; i++)
	{
		for (int j = 0; j < DIM + 1; j++)
		{
			if (elem_new[i].neig[j] == -1)
				continue;
			if (elem_new[i].neig[j] == su_mesh->elem_num)
			{
				elem_new[i].neig[j] = elemNum_include_edge.at(2);
				su_mesh->elem.at(elemNum_include_edge.at(i)).neig[j] = elemNum_include_edge.at(2);
			}
			value_tp = mesh_process.Face_Opposite_Node(su_mesh->elem.at(elem_new[i].neig[j]),
				mesh_process.Node_Opposite_Face(elem_new[i], elem_new[i].form[j]));
			if (value_tp == -1)
			{
				std::cout << "Face_Transform_32 run error, please check the "
					"program!\n";
				exit(-1);
			}
			su_mesh->elem.at(elem_new[i].neig[j]).neig[value_tp] = elemNum_include_edge.at(i);
		}
		// 修改单元所有节点的elem值
		mesh_process.Renew_NodeElem(su_mesh, elemNum_include_edge.at(i));
		// 修改单元质量
		su_mesh->elem.at(elemNum_include_edge.at(i)).quality =
			abs(Shape_Quality(su_mesh->node.at(su_mesh->elem.at(elemNum_include_edge.at(i)).form[0]),
				su_mesh->node.at(su_mesh->elem.at(elemNum_include_edge.at(i)).form[1]),
				su_mesh->node.at(su_mesh->elem.at(elemNum_include_edge.at(i)).form[2]),
				su_mesh->node.at(su_mesh->elem.at(elemNum_include_edge.at(i)).form[3])));
	}
	return elemNum_modify;
}

double _QUALITY::Face_Transform_44_Legality(_SU_MESH* su_mesh, EDGE edge_tp, std::vector<int> elemNum_include_edge)
{
	// //
	// 面交换的合法性判断其实就是比较交换前后的网格单元总体积，若其差值大于设定阈值，则说明网格当前交换不合法，返回-1
	// ELEM elem_tp[4];
	// double all_volume_before = 0;    // 储存面交换前网格单元总体积
	// double all_volume_after = 0;     // 储存面交换后网格单元总体积
	// double min_quality_before = 1.0; //
	// 储存面交换前交换域内最小的网格单元质量 double min_quality_after = 1.0; //
	// 储存面交换后交换域内最小的网格单元质量 std::vector<int> nodeNum; //
	// 声明一个容器，储存交换域的所有网格节点编号 _DATA_PROCESS data_process;
	// // 取出elemNum_include_edge容器内所有网格单元
	// for (int i = 0; i < 4; i++)
	// {
	//   elem_tp[i] = su_mesh->elem.at(elemNum_include_edge.at(i));
	//   all_volume_before +=
	//   abs(data_process.tetrahedral_volume(su_mesh->node.at(elem_tp[i].form[0]),
	//                                                            su_mesh->node.at(elem_tp[i].form[1]),
	//                                                            su_mesh->node.at(elem_tp[i].form[2]),
	//                                                            su_mesh->node.at(elem_tp[i].form[3])));
	//   for (int j = 0; j < DIM + 1; j++)
	//     if (std::find(nodeNum.begin(), nodeNum.end(), elem_tp[i].form[j]) ==
	//     nodeNum.end())
	//       nodeNum.push_back(elem_tp[i].form[j]);
	//   if (min_quality_before > elem_tp[i].quality)
	//     min_quality_before = elem_tp[i].quality;
	// }
	// std::vector<int> nodeNum_gen;  // 声明一个容器，储存进行面交换后，
	// int cnt = 0;
	// for (std::vector<int>::iterator iter = nodeNum.begin(); iter !=
	// nodeNum.end(); ++iter)
	//   if (*iter != edge_tp.form[0] && *iter != edge_tp.form[1])
	//     face_tp.form[cnt++] = *iter;
	// face_tp.Sort();
	// for (int i = 0; i < 2; i++)
	//   all_volume_after +=
	//   abs(data_process.tetrahedral_volume(su_mesh->node.at(face_tp.form[0]),
	//                                                           su_mesh->node.at(face_tp.form[1]),
	//                                                           su_mesh->node.at(face_tp.form[2]),
	//                                                           su_mesh->node.at(edge_tp.form[i])));
	// // 若其差值大于设定阈值，则说明网格当前交换不合法，返回-1
	// if (abs(all_volume_after - all_volume_before) > Delta_volume)
	//   return -1;
	// // 储存进行面交换后形成的局部交换域内的最低网格质量
	// double quality_tp;
	// for (int i = 0; i < 2; i++)
	// {
	//   quality_tp = abs(Shape_Quality(su_mesh->node.at(face_tp.form[0]),
	//                                  su_mesh->node.at(face_tp.form[1]),
	//                                  su_mesh->node.at(face_tp.form[2]),
	//                                  su_mesh->node.at(edge_tp.form[i])));
	//   if (min_quality_after > quality_tp)
	//     min_quality_after = quality_tp;
	// }
	// if (min_quality_after >= min_quality_before)
	//   return min_quality_after;
	// else
	return -1;
}

std::vector<int> _QUALITY::Face_Transform_44(_SU_MESH* su_mesh, EDGE edge_tp)
{
	std::vector<int> elemNum_include_edge;
	std::vector<int>
		elemNum_modify; // 声明一个int类型容器，储存成功面交换后网格节点有过变动的所有网格单元编号
	return elemNum_modify;
}

double _QUALITY::Face_Transform_56_Legality(_SU_MESH* su_mesh, EDGE edge_tp, std::vector<int> elemNum_include_edge)
{
	return -1;
}

std::vector<int> _QUALITY::Face_Transform_56(_SU_MESH* su_mesh, EDGE edge_tp)
{
	std::vector<int> elemNum_include_edge;
	std::vector<int>
		elemNum_modify; // 声明一个int类型容器，储存成功面交换后网格节点有过变动的所有网格单元编号
	return elemNum_modify;
}

void _QUALITY::Quality_Optimization_Face_Transform(_SU_MESH* su_mesh)
{
	ELEM elem_tp;
	std::deque<int> elemNum_judge; // 声明一个int类型容器，储存待判断的网格单元编号，每次成功进行面交换后，将生成的网格单元编号全部压入该容器内
	// 首先遍历当前三角化，将网格质量低于Minimum_Shape_Quality_Face_Transform的网格单元编号压入elemNum_judge
	for (int i = 0; i < su_mesh->elem_num; ++i)
		if (su_mesh->elem.at(i).quality < Minimum_Shape_Quality_Face_Transform)
			elemNum_judge.push_back(i);
	// 每个网格单元最多会有10种交换情况，6种与网格边有关，4种与网格面有关
	// 首先需要判断每种面交换是否合法，其次比较合法的所有面交换后形成的局部交换域内的最低网格质量，选取最大的那种且能提高原始网格域最低网格质量的交换方法作为最终交换算法
	double min_quality[10];
	double min_quality_judge[10];
	int face_transform_type[10]; // 判断需要进行的面交换种类，1代表无，2代表T23，3代表T32，4代表T44，5代表T56
	EDGE edge_tp[6];
	FACE face_tp[4];
	std::vector<int> elemNum_include_edge; // 声明一个容器，储存所有包含待判断网格边的网格单元编号
	std::vector<int> elemNum_include_face; // 声明一个容器，储存所有包含待判断网格面的网格单元编号
	int cnt = 0;
	std::vector<int> elemNum_modify;       // 声明一个int类型容器，储存成功面交换后网格节点有过变动的所有网格单元编号
	std::deque<int>::iterator elemNum_iter;
	_MESH_PROCESS mesh_process;
	// 只要elemNum_iter不为空，就一直进行循环判断
	while (!elemNum_judge.empty())
	{
		elem_tp = su_mesh->elem.at(elemNum_judge.front());
		elemNum_judge.pop_front();
		cnt = 0; // 计数器归零
		// 先判断每条网格边的交换情况
		for (int i = 0; i < DIM + 1; i++)
			for (int j = i + 1; j < DIM + 1; j++)
			{
				// 跳过边界边
				if (elem_tp.form[i] < su_mesh->InitNode_num && elem_tp.form[j] < su_mesh->InitNode_num)
				{
					min_quality[cnt] = -1;
					face_transform_type[cnt++] = 1;
					continue;
				}
				std::vector<int>().swap(elemNum_include_edge); // 初始化elemNum_include_edge，并释放容器空间
				edge_tp[cnt].form[0] = elem_tp.form[i];
				edge_tp[cnt].form[1] = elem_tp.form[j];
				mesh_process.FindRing(su_mesh, edge_tp[cnt], &elemNum_include_edge, "fast");
				switch (elemNum_include_edge.size())
				{
				case 1:
					std::cout << "FindRing run error!\n";
					exit(-1);
					break;
				case 2:
					std::cout << "FindRing run error!\n";
					exit(-1);
					break;
				case 3:
					if ((min_quality[cnt] = Face_Transform_32_Legality(su_mesh, edge_tp[cnt], elemNum_include_edge)) != -1)
						face_transform_type[cnt++] = 3;
					else
						face_transform_type[cnt++] = 1;
					break;
				case 4:
					if ((min_quality[cnt] = Face_Transform_44_Legality(su_mesh, edge_tp[cnt], elemNum_include_edge)) != -1)
						face_transform_type[cnt++] = 4;
					else
						face_transform_type[cnt++] = 1;
					break;
				case 5:
					if ((min_quality[cnt] = Face_Transform_56_Legality(su_mesh, edge_tp[cnt], elemNum_include_edge)) != -1)
						face_transform_type[cnt++] = 5;
					else
						face_transform_type[cnt++] = 1;
					break;
				default:
					min_quality[cnt] = -1;
					face_transform_type[cnt++] = 1;
					break;
				}
			}
		// 再判断每条网格面的交换情况
		for (int i = 0; i < DIM + 1; i++)
			for (int j = i + 1; j < DIM + 1; j++)
				for (int k = j + 1; k < DIM + 1; k++)
				{
					// 跳过边界面
					if (elem_tp.form[i] < su_mesh->InitNode_num && elem_tp.form[j] < su_mesh->InitNode_num && elem_tp.form[k] < su_mesh->InitNode_num)
					{
						min_quality[cnt] = -1;
						face_transform_type[cnt++] = 1;
						continue;
					}
					std::vector<int>().swap(elemNum_include_face); // 初始化elemNum_include_face，并释放容器空间
					face_tp[cnt - 6].form[0] = elem_tp.form[i];
					face_tp[cnt - 6].form[1] = elem_tp.form[j];
					face_tp[cnt - 6].form[2] = elem_tp.form[k];
					mesh_process.FindAwl(su_mesh, face_tp[cnt - 6], &elemNum_include_face, "fast");
					switch (elemNum_include_face.size())
					{
					case 1:
						std::cout << "FindAwl run error!\n";
						exit(-1);
						break;
					case 2:
						if ((min_quality[cnt] = Face_Transform_23_Legality(su_mesh, face_tp[cnt - 6], elemNum_include_face)) != -1)
							face_transform_type[cnt++] = 2;
						else
							face_transform_type[cnt++] = 1;
						break;
					default:
						min_quality[cnt] = -1;
						face_transform_type[cnt++] = 1;
						break;
					}
				}
		// 在min_quality内查找最大的网格质量，接受能实现该网格质量的那种面交换算法
		std::copy(min_quality, min_quality + 10, min_quality_judge);
		std::sort(min_quality_judge, min_quality_judge + 10);
		// 如果所有值都为-1，代表不需要进行面交换，直接continue
		if (min_quality_judge[9] == -1)
			continue;
		for (cnt = 0; cnt < 10; cnt++)
			if (min_quality[cnt] == min_quality_judge[9])
				break;
		std::vector<int>().swap(elemNum_modify); // 初始化elemNum_include_edge，并释放容器空间
		switch (face_transform_type[cnt])
		{
		case 1:
			break;
		case 2:
			elemNum_modify = Face_Transform_23(su_mesh, face_tp[cnt - 6]);
			break;
		case 3:
			elemNum_modify = Face_Transform_32(su_mesh, edge_tp[cnt]);
			// T32删除了一个网格单元
			if ((elemNum_iter = std::find(elemNum_judge.begin(), elemNum_judge.end(), su_mesh->elem_num)) != elemNum_judge.end())
				elemNum_judge.erase(elemNum_iter);
			break;
		case 4:
			elemNum_modify = Face_Transform_44(su_mesh, edge_tp[cnt]);
			break;
		case 5:
			elemNum_modify = Face_Transform_56(su_mesh, edge_tp[cnt]);
			break;
		default:
			break;
		}
		for (std::vector<int>::iterator iter = elemNum_modify.begin(); iter != elemNum_modify.end(); ++iter)
			if (std::find(elemNum_judge.begin(), elemNum_judge.end(), *iter) == elemNum_judge.end())
				elemNum_judge.push_back(*iter);
	}
	return;
}

void _QUALITY::check(_SU_MESH* su_mesh)
{
	std::vector<double> volume;
	std::vector<double> Shape_Quality;
	double all_volume = 0;
	int i = 0;
	int cnt = 0;
	_DATA_PROCESS data_process;
	for (std::vector<ELEM>::iterator elem_iter = su_mesh->elem.begin();
		elem_iter != su_mesh->elem.end();
		++elem_iter)
	{
		volume.push_back(abs(data_process.tetrahedral_volume(
			su_mesh->node.at(elem_iter->form[0]),
			su_mesh->node.at(elem_iter->form[1]),
			su_mesh->node.at(elem_iter->form[2]),
			su_mesh->node.at(elem_iter->form[3]))));
		all_volume += volume.back();
		if (elem_iter->quality < 0.01)
		{
			cnt++;
			std::cout << i << '\n';
		}
		if (elem_iter->quality == NAN)
			std::cout << i << '\n';
		Shape_Quality.push_back(elem_iter->quality);
		i++;
	}
	if (cnt > 0)
		std::cout << cnt << ' ';
	std::sort(volume.begin(), volume.end());
	std::sort(Shape_Quality.begin(), Shape_Quality.end());
	std::cout << std::fixed << all_volume << ' ';
	return;
}

bool _QUALITY::Optimization_based_Smoothing_Compared(_SU_MESH* su_mesh, int Optimization_based_Smoothing_nodeNum)
{
	double target_function_value_before = 0; // 定义一个变量，储存上次迭代后目标函数值，初始化为0
	double target_function_value_after = 0;  // 定义一个变量，储存本次迭代后目标函数值，初始化为0
	// * 首先得到局部优化域的初始目标函数值
	std::vector<int> elemNum_IncludeNode; // 声明一个容器，用来储存包含待优化节点的所有网格单元编号
	_MESH_PROCESS mesh_process;
	_DATA_PROCESS data_process;
	// 查找包含待优化节点的所有网格单元编号
	mesh_process.FindBall_fast(su_mesh, Optimization_based_Smoothing_nodeNum, &elemNum_IncludeNode);
	// 目标函数为该节点相邻单元的所有错误函数相加，此处直接累加所有单元的质量倒数即可
	for (std::vector<int>::iterator iter = elemNum_IncludeNode.begin(); iter != elemNum_IncludeNode.end(); ++iter)
		target_function_value_before += 1 / su_mesh->elem.at(*iter).quality;
	// * 储存局部优化域的边界面
	std::vector<FACE> face_Optimization_Domain; // 创建一个容器，用来储存局部优化域的边界面
	FACE face_tp;
	// 将所有局部优化域的边界面压入face_Optimization_Domain，并调整边节点顺序，使得以
	// 面点1、面点2、面点3、待优化点的顺序是orient3D规定的正方向
	// 便于后续判断待优化点Optimization_based光顺后是否有效
	for (std::vector<int>::iterator iter = elemNum_IncludeNode.begin(); iter != elemNum_IncludeNode.end(); ++iter)
	{
		switch (mesh_process.ELEM_Include_Node(su_mesh->elem.at(*iter), Optimization_based_Smoothing_nodeNum))
		{
		case 0:
			face_tp.form[0] = su_mesh->elem.at(*iter).form[1];
			face_tp.form[1] = su_mesh->elem.at(*iter).form[2];
			face_tp.form[2] = su_mesh->elem.at(*iter).form[3];
			break;
		case 1:
			face_tp.form[0] = su_mesh->elem.at(*iter).form[0];
			face_tp.form[1] = su_mesh->elem.at(*iter).form[2];
			face_tp.form[2] = su_mesh->elem.at(*iter).form[3];
			break;
		case 2:
			face_tp.form[0] = su_mesh->elem.at(*iter).form[0];
			face_tp.form[1] = su_mesh->elem.at(*iter).form[1];
			face_tp.form[2] = su_mesh->elem.at(*iter).form[3];
			break;
		case 3:
			face_tp.form[0] = su_mesh->elem.at(*iter).form[0];
			face_tp.form[1] = su_mesh->elem.at(*iter).form[1];
			face_tp.form[2] = su_mesh->elem.at(*iter).form[2];
			break;
		default:
			break;
		}
		// 使得以 面点1、面点2、面点3、待优化点的顺序是orient3D规定的正方向
		if (data_process.tetrahedral_volume(su_mesh->node.at(face_tp.form[0]),
			su_mesh->node.at(face_tp.form[1]),
			su_mesh->node.at(face_tp.form[2]),
			su_mesh->node.at(Optimization_based_Smoothing_nodeNum)) < 0)
			std::swap(face_tp.form[1], face_tp.form[2]);
		face_Optimization_Domain.push_back(face_tp);                               // 压入face_Optimization_Domain
	}
	NODE node_original = su_mesh->node.at(Optimization_based_Smoothing_nodeNum); // 定义一个NODE类，作为原始点，即程序输入的待优化点
	NODE node_before = node_original;                                            // 声明一个NODE类，储存本次迭代前点位置，初始化为原始点位置
	NODE node_after;                                                             // 声明一个NODE类，储存本次迭代后点位置
	NODE node_Delta;                                                             // 声明一个NODE类，作为每次迭代的节点坐标变化值
	double node_delta[3][1];
	double gradient_before[3][1]{};
	double gradient_now[3][1]{};
	double gradient_delta[3][1]{};
	double H_before[3][3]{};
	double H_now[3][3]{};
	// 声明一个变量，作为当前迭代位置的最速下降方向
	double steepest_descent[3][1];
	double steepest_descent_Module; // 最速下降方向向量的模
	double optimal_step = 0;        // 定义一个变量，作为每次迭代的类最优步长，并且下次迭代的初始步长取为该值，初始化为0
	int iter = 0;                   // 声明一个变量，作为迭代次数
	do
	{
		// 储存上次迭代结束后目标函数值，第一次迭代时，不需要储存
		if (target_function_value_after != 0) target_function_value_before = target_function_value_after;
		// 先由当前点位置求目标函数当前位置梯度
		for (int i = 0; i < 3; i++)
		{
			gradient_before[i][0] = gradient_now[i][0];
			if ((gradient_now[i][0] = target_Function_first_derivative(su_mesh, node_before, face_Optimization_Domain, i)) == -1)
			{
				std::cout << "target_Function_first_derivative value =-1!\n";
				exit(-1);
			}
		}
		// 第一次循环计算hessian矩阵
		if (iter == 0)
		{
			for (int i = 0; i < 3; i++)
				for (int j = 0; j < 3; j++)
					if ((H_now[i][j] = target_Function_second_derivative(su_mesh, node_before, face_Optimization_Domain, i, j)) == -1)
					{
						std::cout << "target_Function_second_derivative value =-1!\n";
						exit(-1);
					}
		}
		else {
			// 计算本次迭代与上次梯度差值
			for (int i = 0; i < 3; i++)
				gradient_delta[i][0] = gradient_now[i][0] - gradient_before[i][0];
			for (int i = 0; i < 3; i++)
				for (int j = 0; j < 3; j++)
					H_before[i][j] = H_now[i][j];
			// 计算本次迭代的类hessian矩阵
			Compute_HessianLike_matrice(H_before, node_delta, gradient_delta, H_now);
		}
		// 求当前位置最速下降方向
		data_process.matrix_product(H_now, gradient_now, steepest_descent);
		steepest_descent[0][0] = -steepest_descent[0][0];
		steepest_descent[1][0] = -steepest_descent[1][0];
		steepest_descent[2][0] = -steepest_descent[2][0];
		// 求最速下降方向向量的模
		steepest_descent_Module = data_process.Vector_Module(steepest_descent);
		// 最速下降方向向量的模小于最小最速下降方向模时代表达到极值点处，停止迭代
		if (steepest_descent_Module < Min_steepest_descent)
			break;
		// 为避免不必要的误差，每一次迭代时，需要根据最速下降方向梯度的模来自适应地确定第一次迭代的初始步长
		// 设定为在当前最速下降方向梯度下，待优化节点最大移动长度为理想的最大边边长
		optimal_step = su_mesh->longest_distance / steepest_descent_Module;
		// 采用强Wolfe准则，利用牛顿方向，利用上次迭代的最优步长的两倍作为本次迭代的初始步长
		optimal_step = Line_Search_Algorithm_with_Wolfe(su_mesh, node_before, face_Optimization_Domain, steepest_descent, optimal_step);
		if (optimal_step == 0)
			break;
		// 求本次迭代节点坐标变化
		for (int i = 0; i < 3; i++)
			node_delta[i][0] = optimal_step * steepest_descent[i][0];
		node_Delta = get_Node_Delta(optimal_step, steepest_descent);
		node_after = node_before + node_Delta;
		target_function_value_after = target_Function(su_mesh, node_after, face_Optimization_Domain);
		if (target_function_value_after == -1)
		{
			std::cout << "target_function_value_after=-1\n";
			exit(-1);
		}
		node_before = node_after;
		iter++;
	} while (optimal_step > Min_step && iter < Max_iter && abs(target_function_value_after - target_function_value_before) > Min_imp);
	su_mesh->node.at(Optimization_based_Smoothing_nodeNum) = node_before;
	// 修改局部优化域内每一个网格单元的单元质量
	for (std::vector<int>::iterator iter = elemNum_IncludeNode.begin(); iter != elemNum_IncludeNode.end(); ++iter)
		su_mesh->elem.at(*iter).quality = abs(Shape_Quality(su_mesh->node.at(su_mesh->elem.at(*iter).form[0]),
			su_mesh->node.at(su_mesh->elem.at(*iter).form[1]),
			su_mesh->node.at(su_mesh->elem.at(*iter).form[2]),
			su_mesh->node.at(su_mesh->elem.at(*iter).form[3])));
	return true;
}


void _QUALITY::Compute_HessianLike_matrice(double matrix_1[3][3], double matrix_2[3][1], double matrix_3[3][1], double matrix_target[3][3])
{
	_DATA_PROCESS data_process;
	double matrix_2_transpose[3];
	double matrix_3_transpose[3];
	// 计算两个矩阵向量的转置
	data_process.Matrix_transpose(matrix_2, matrix_2_transpose);
	data_process.Matrix_transpose(matrix_3, matrix_3_transpose);
	double matrix_pro_1[3][3];
	double matrix_pro_2[3][3];
	data_process.matrix_product(matrix_2, matrix_2_transpose, matrix_pro_1);
	data_process.matrix_product(matrix_1, matrix_3, matrix_3_transpose, matrix_1, matrix_pro_2);
	double product_1, product_2;
	product_1 = data_process.matrix_product(matrix_2_transpose, matrix_3);
	product_2 = data_process.matrix_product(matrix_3_transpose, matrix_1, matrix_3);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			matrix_target[i][j] = matrix_1[i][j] + matrix_pro_1[i][j] / product_1 + matrix_pro_2[i][j] / product_2;
	return;
}
