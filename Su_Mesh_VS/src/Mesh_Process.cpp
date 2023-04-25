#include "Su_Mesh.h"

int _MESH_PROCESS::Elem_Include_Node(ELEM elem_tp, int nodeNum_tp)
{
	for (int i = 0; i < DIM + 1; i++)
		if (elem_tp.form[i] == nodeNum_tp)
			return i;
	return -1;
}

int _MESH_PROCESS::Face_Opposite_Node(ELEM elem_tp, FACE face_tp)
{
	face_tp.Sort();
	// 由于任意一个网格面的三个节点总是按节点编号从小到大排序，所以可以直接判断
	if (elem_tp.form[0] == face_tp.form[0] && elem_tp.form[1] == face_tp.form[1] && elem_tp.form[2] == face_tp.form[2])
		return 3;
	else if (elem_tp.form[0] == face_tp.form[0] && elem_tp.form[1] == face_tp.form[1] && elem_tp.form[3] == face_tp.form[2])
		return 2;
	else if (elem_tp.form[0] == face_tp.form[0] && elem_tp.form[2] == face_tp.form[1] && elem_tp.form[3] == face_tp.form[2])
		return 1;
	else if (elem_tp.form[1] == face_tp.form[0] && elem_tp.form[2] == face_tp.form[1] && elem_tp.form[3] == face_tp.form[2])
		return 0;
	return -1;
}

FACE _MESH_PROCESS::Node_Opposite_Face(ELEM elem_tp, int nodeNum_tp)
{
	FACE face_tp;
	switch (Elem_Include_Node(elem_tp, nodeNum_tp))
	{
	case 0:
		face_tp.form[0] = elem_tp.form[1];
		face_tp.form[1] = elem_tp.form[2];
		face_tp.form[2] = elem_tp.form[3];
		break;
	case 1:
		face_tp.form[0] = elem_tp.form[0];
		face_tp.form[1] = elem_tp.form[2];
		face_tp.form[2] = elem_tp.form[3];
		break;
	case 2:
		face_tp.form[0] = elem_tp.form[0];
		face_tp.form[1] = elem_tp.form[1];
		face_tp.form[2] = elem_tp.form[3];
		break;
	case 3:
		face_tp.form[0] = elem_tp.form[0];
		face_tp.form[1] = elem_tp.form[1];
		face_tp.form[2] = elem_tp.form[2];
		break;
	default:
		break;
	}
	return face_tp;
}

int _MESH_PROCESS::ELEM_Include_Node(ELEM elem_tp, int nodeNum_tp)
{
	for (int i = 0; i < DIM + 1; i++)
		if (elem_tp.form[i] == nodeNum_tp)
			return i;
	return -1;
}

EDGE _MESH_PROCESS::Edge_Opposite_Edge(ELEM elem_tp, EDGE edge_tp)
{
	EDGE edge;
	if (elem_tp.form[0] == edge_tp.form[0])
	{
		if (elem_tp.form[1] == edge_tp.form[1])
		{
			edge.form[0] = elem_tp.form[2];
			edge.form[1] = elem_tp.form[3];
		}
		else if (elem_tp.form[2] == edge_tp.form[1])
		{
			edge.form[0] = elem_tp.form[1];
			edge.form[1] = elem_tp.form[3];
		}
		else
		{
			edge.form[0] = elem_tp.form[1];
			edge.form[1] = elem_tp.form[2];
		}
	}
	else if (elem_tp.form[1] == edge_tp.form[0])
	{
		if (elem_tp.form[2] == edge_tp.form[1])
		{
			edge.form[0] = elem_tp.form[0];
			edge.form[1] = elem_tp.form[3];
		}
		else
		{
			edge.form[0] = elem_tp.form[0];
			edge.form[1] = elem_tp.form[2];
		}
	}
	else
	{
		edge.form[0] = elem_tp.form[0];
		edge.form[1] = elem_tp.form[1];
	}
	return edge;
}

int _MESH_PROCESS::FACE_Include_Node(FACE face_tp, int nodeNum_tp)
{
	for (int i = 0; i < DIM; i++)
		if (face_tp.form[i] == nodeNum_tp)
			return i;
	return -1;
}

EDGE _MESH_PROCESS::Node_Opposite_Edge(FACE face_tp, int nodeNum_tp)
{
	EDGE edge_tp;
	switch (FACE_Include_Node(face_tp, nodeNum_tp))
	{
	case 0:
		edge_tp.form[0] = face_tp.form[1];
		edge_tp.form[1] = face_tp.form[2];
		break;
	case 1:
		edge_tp.form[0] = face_tp.form[0];
		edge_tp.form[1] = face_tp.form[2];
		break;
	case 2:
		edge_tp.form[0] = face_tp.form[0];
		edge_tp.form[1] = face_tp.form[1];
		break;
	default:
		break;
	}
	return edge_tp;
}

bool _MESH_PROCESS::Elem_Adjacent(ELEM elem_tp1, ELEM elem_tp2)
{
	// 读入8个节点编号
	int arr[] = { elem_tp1.form[0], elem_tp1.form[1], elem_tp1.form[2], elem_tp1.form[3], elem_tp2.form[0], elem_tp2.form[1], elem_tp2.form[2], elem_tp2.form[3] };
	// 对着8个节点编号进行排序，便于判断相同节点编号的组数
	std::sort(arr, arr + 8);
	// 计数器，记录有几组相同的节点编号
	int cnt = 0;
	for (int i = 0; i < 7; i++)
	{
		if (arr[i + 1] == arr[i])
			cnt++;
	}
	if (cnt == 3)
		return true;
	else
		return false;
}

int _MESH_PROCESS::AdjacentElem_pos(ELEM elem_tp, int elemNum_tp)
{
	for (int i = 0; i < DIM + 1; i++)
		if (elem_tp.neig[i] == elemNum_tp)
			return i;
	return -1;
}

FACE _MESH_PROCESS::elem_AdjacentFace(ELEM elem_tp, int elemNum_tp)
{
	FACE face_tp; // 声明一个变量，储存相邻边
	// 判断该网格单元编号所代表的网格单元在给定网格单元的neig中位置，来判断相邻边
	switch (AdjacentElem_pos(elem_tp, elemNum_tp))
	{
	case 0:
		face_tp.form[0] = elem_tp.form[1];
		face_tp.form[1] = elem_tp.form[2];
		face_tp.form[2] = elem_tp.form[3];
		break;
	case 1:
		face_tp.form[0] = elem_tp.form[0];
		face_tp.form[1] = elem_tp.form[2];
		face_tp.form[2] = elem_tp.form[3];
		break;
	case 2:
		face_tp.form[0] = elem_tp.form[0];
		face_tp.form[1] = elem_tp.form[1];
		face_tp.form[2] = elem_tp.form[3];
		break;
	case 3:
		face_tp.form[0] = elem_tp.form[0];
		face_tp.form[1] = elem_tp.form[1];
		face_tp.form[2] = elem_tp.form[2];
	default:
		break;
	}
	return face_tp;
}

FACE _MESH_PROCESS::elem_AdjacentFace(ELEM elem_tp1, ELEM elem_tp2)
{
	FACE face_tp;
	// 读入8个节点编号
	int arr[] = { elem_tp1.form[0], elem_tp1.form[1], elem_tp1.form[2], elem_tp1.form[3], elem_tp2.form[0], elem_tp2.form[1], elem_tp2.form[2], elem_tp2.form[3] };
	// 对这8个节点编号进行排序，便于判断相同节点编号的组数
	std::sort(arr, arr + 8);
	// 计数器，记录有几组相同的节点编号
	int cnt = 0;
	for (int i = 0; i < 7; i++)
	{
		if (arr[i + 1] == arr[i])
			face_tp.form[cnt++] = arr[i];
	}
	return face_tp;
}

void _MESH_PROCESS::Renew_NodeElem(_SU_MESH* su_mesh, int elemNum_tp)
{
	for (int i = 0; i < DIM + 1; i++)
		su_mesh->node.at(su_mesh->elem.at(elemNum_tp).form[i]).elem = elemNum_tp;
	return;
}

int _MESH_PROCESS::Find_OneElem_IncludeNode_nodeNum(_SU_MESH* su_mesh, int nodeNum, NODE node_tp)
{
	ELEM elem_tp;
	int elemNum_tp;
	std::vector<int> elemNum_wait; // 创建一个容器，用来储存待判断的网格单元
	std::vector<int> elemNum_succ; // 创建一个容器，用来储存判断过的单元
	double judgment;
	// 如果当前插入的是第一个边界点，则将0插入elemNum_wait
	elemNum_wait.push_back(nodeNum == -1 ? 0 : su_mesh->node.at(nodeNum).elem); // 压入初始网格单元
	_DATA_PROCESS data_process;
	while (!elemNum_wait.empty())
	{
		elemNum_tp = elemNum_wait.front(); // 取出elemNum_wait中的第一个元素并擦除
		elemNum_wait.erase(elemNum_wait.begin());
		elem_tp = su_mesh->elem.at(elemNum_tp);
		judgment = data_process.in_sphere(su_mesh->node.at(elem_tp.form[0]),
			su_mesh->node.at(elem_tp.form[1]),
			su_mesh->node.at(elem_tp.form[2]),
			su_mesh->node.at(elem_tp.form[3]),
			node_tp);
		if (judgment > 0)
			return elemNum_tp;
		if (judgment == 0)
			return -2;
		elemNum_succ.push_back(elemNum_tp);
		// 将该网格单元周围未被判断过的相邻单元压入elemNum_wait
		for (int i = 0; i < DIM + 1; i++)
			if (elem_tp.neig[i] != -1)
				// 判断该单元是否存在于elemNum_wait，如不存在再往下判断
				// 在elemNum_succ内查找elemNum_tp对应网格单元相邻网格单元，如果搜到end()，代表没有找到，即该网格没被判断过
				if (std::find(elemNum_wait.begin(), elemNum_wait.end(), elem_tp.neig[i]) == elemNum_wait.end())
					if (std::find(elemNum_succ.begin(), elemNum_succ.end(), elem_tp.neig[i]) == elemNum_succ.end())
						elemNum_wait.push_back(elem_tp.neig[i]);
	}
	return -1;
}

int _MESH_PROCESS::Find_OneElem_IncludeNode_elemNum(_SU_MESH* su_mesh, int elemNum, NODE node_tp)
{
	ELEM elem_tp;
	int elemNum_tp;
	std::vector<int> elemNum_wait; // 创建一个容器，用来储存待判断的网格单元
	std::vector<int> elemNum_succ; // 创建一个容器，用来储存判断过的单元
	double judgment;
	// 首先判断elem.at(elemNum)是否有效
	if (std::find(su_mesh->elemNum_invalid.begin(), su_mesh->elemNum_invalid.end(), elemNum) != su_mesh->elemNum_invalid.end())
		elemNum = su_mesh->node.back().elem;
	elemNum_wait.push_back(elemNum); // 压入初始网格单元
	_DATA_PROCESS data_process;
	while (!elemNum_wait.empty())
	{
		elemNum_tp = elemNum_wait.front(); // 取出elemNum_wait中的第一个元素并擦除
		elemNum_wait.erase(elemNum_wait.begin());
		elem_tp = su_mesh->elem.at(elemNum_tp);
		judgment = data_process.in_sphere(su_mesh->node.at(elem_tp.form[0]), su_mesh->node.at(elem_tp.form[1]), su_mesh->node.at(elem_tp.form[2]), su_mesh->node.at(elem_tp.form[3]), node_tp);
		if (judgment > 0)
			return elemNum_tp;
		if (judgment == 0)
			return -2;
		elemNum_succ.push_back(elemNum_tp);
		// 将该网格单元周围未被判断过的相邻单元压入elemNum_wait
		for (int i = 0; i < DIM + 1; i++)
			if (elem_tp.neig[i] != -1)
				// 判断该单元是否存在于elemNum_wait，如不存在再往下判断
				// 在elemNum_succ内查找elemNum_tp对应网格单元相邻网格单元，如果搜到end()，代表没有找到，即该网格没被判断过
				if (std::find(elemNum_wait.begin(), elemNum_wait.end(), elem_tp.neig[i]) == elemNum_wait.end())
					if (std::find(elemNum_succ.begin(), elemNum_succ.end(), elem_tp.neig[i]) == elemNum_succ.end())
						elemNum_wait.push_back(elem_tp.neig[i]);
	}
	return -1;
}

void _MESH_PROCESS::FindBall_fast(_SU_MESH* su_mesh, int nodeNum_tp, std::vector<int>* elemNum_IncludeNode)
{
	ELEM elem_tp;                                                      // 声明一个变量，用于临时储存待判断网格单元
	int elemNum_tp;
	std::vector<int> elemNum_wait;                                     // 创建一个容器，用来储存待判断的网格单元
	std::vector<int> elemNum_succ;                                     // 创建一个容器，用来储存判断过的单元
	elemNum_IncludeNode->push_back(su_mesh->node.at(nodeNum_tp).elem); // 压入该节点的elem值，即已知包含该节点的网格单元编号
	elemNum_wait.push_back(su_mesh->node.at(nodeNum_tp).elem);         // 压入该节点的elem值，即已知包含该节点的网格单元编号
	while (!elemNum_wait.empty())
	{
		elemNum_tp = elemNum_wait.front(); // 取出elemNum_wait中的第一个元素并擦除
		elemNum_wait.erase(elemNum_wait.begin());
		elem_tp = su_mesh->elem.at(elemNum_tp);
		for (int i = 0; i < DIM + 1; i++) // 检索elem_tp单元周围的单元
		{
			// 如果elem_tp单元周围存在编号为-1的单元，则直接跳过该单元
			if (elem_tp.neig[i] == -1)
				continue;
			// 如果elem_tp单元周围的单元未被判断过并且不在当前待判断列表中，并且该单元包含nodeNum_tp，则将该单元压入elemNum_IncludeNode中
			if ((std::find(elemNum_wait.begin(), elemNum_wait.end(), elem_tp.neig[i]) == elemNum_wait.end()) &&
				(std::find(elemNum_succ.begin(), elemNum_succ.end(), elem_tp.neig[i]) == elemNum_succ.end()) &&
				(Elem_Include_Node(su_mesh->elem.at(elem_tp.neig[i]), nodeNum_tp) != -1))
			{
				elemNum_IncludeNode->push_back(elem_tp.neig[i]);
				elemNum_wait.push_back(elem_tp.neig[i]);
			}
		}
		elemNum_succ.push_back(elemNum_tp);
	}
	return;
}

void _MESH_PROCESS::FindBall_slow(_SU_MESH* su_mesh, int nodeNum_tp, std::vector<int>* elemNum_IncludeNode)
{
	int cnt = 0; // 定义一个变量，用于得到当前查找单元编号
	for (std::vector<ELEM>::iterator elem_iter = su_mesh->elem.begin(); elem_iter != su_mesh->elem.end(); ++elem_iter)
	{
		if (Elem_Include_Node(*elem_iter, nodeNum_tp) != -1)
			elemNum_IncludeNode->push_back(cnt);
		cnt++;
	}
	return;
}

void _MESH_PROCESS::FindRing(_SU_MESH* su_mesh, EDGE edge_tp, std::vector<int>* elemNum_IncludeEdge, std::string judgment)
{
	std::vector<int> elemNum_IncludeNode;                            // 声明一个容器，用来储存包含某节点的所有网格单元
	if (judgment == "fast")
		FindBall_fast(su_mesh, edge_tp.form[0], &elemNum_IncludeNode); // 查找包含face_tp网格面第一个节点的所有网格单元
	else if (judgment == "slow")
		FindBall_slow(su_mesh, edge_tp.form[0], &elemNum_IncludeNode); // 查找包含face_tp网格面第一个节点的所有网格单元
	// 检索elemNum_IncludeNode容器内所有值，查找包含edge_tp网格边的剩下所有节点的网格单元编号
	for (std::vector<int>::iterator iter = elemNum_IncludeNode.begin(); iter != elemNum_IncludeNode.end(); ++iter)
		if ((Elem_Include_Node(su_mesh->elem.at(*iter), edge_tp.form[1]) != -1))
			elemNum_IncludeEdge->push_back(*iter);
}

void _MESH_PROCESS::FindAwl(_SU_MESH* su_mesh, FACE face_tp, std::vector<int>* elemNum_IncludeFace, std::string judgment)
{
	std::vector<int> elemNum_IncludeNode;                            // 声明一个容器，用来储存包含某节点的所有网格单元
	if (judgment == "fast")
		FindBall_fast(su_mesh, face_tp.form[0], &elemNum_IncludeNode); // 查找包含face_tp网格面第一个节点的所有网格单元
	else if (judgment == "slow")
		FindBall_slow(su_mesh, face_tp.form[0], &elemNum_IncludeNode); // 查找包含face_tp网格面第一个节点的所有网格单元
	// 检索elemNum_IncludeNode容器内所有值，查找包含face_tp网格面的剩下所有节点的网格单元编号
	for (std::vector<int>::iterator iter = elemNum_IncludeNode.begin(); iter != elemNum_IncludeNode.end(); ++iter)
		if ((Elem_Include_Node(su_mesh->elem.at(*iter), face_tp.form[1]) != -1) &&
			(Elem_Include_Node(su_mesh->elem.at(*iter), face_tp.form[2]) != -1))
			elemNum_IncludeFace->push_back(*iter);
	return;
}

bool _MESH_PROCESS::Update_Djacency(_SU_MESH* su_mesh, FACE face_tp, std::string judgment)
{
	std::vector<int> elemNum_IncludeFace;                        // 声明一个容器，荣来储存包含face_tp网格面的网格单元编号
	FindAwl(su_mesh, face_tp, &elemNum_IncludeFace, judgment); // 查找包含face_tp网格面的所有网格单元
	_SHEWCHUK shewchuk;
	if (elemNum_IncludeFace.size() > 2)                          // 正常情况下问题域内部一个网格面只会同时被两个网格单元包含，若超过则代表网格生成中出了问题
	{
		std::cout << "Update djacency error: the face formed by node " << face_tp.form[0] << " , "
			<< face_tp.form[1] << " and node " << face_tp.form[2] << std::endl;
		return false;
	}
	else if (elemNum_IncludeFace.size() == 2) // 正常边界面
	{
		// 判断是否存在网格交叉情况
		FACE face_tp = elem_AdjacentFace(su_mesh->elem.at(elemNum_IncludeFace.at(0)), su_mesh->elem.at(elemNum_IncludeFace.at(1)));
		double tp1 = shewchuk.orient3d(su_mesh->node.at(face_tp.form[0]).pos,
			su_mesh->node.at(face_tp.form[1]).pos,
			su_mesh->node.at(face_tp.form[2]).pos,
			su_mesh->node.at(su_mesh->elem.at(elemNum_IncludeFace.at(0)).form[Face_Opposite_Node(su_mesh->elem.at(elemNum_IncludeFace.at(0)), face_tp)]).pos);
		double tp2 = shewchuk.orient3d(su_mesh->node.at(face_tp.form[0]).pos,
			su_mesh->node.at(face_tp.form[1]).pos,
			su_mesh->node.at(face_tp.form[2]).pos,
			su_mesh->node.at(su_mesh->elem.at(elemNum_IncludeFace.at(1)).form[Face_Opposite_Node(su_mesh->elem.at(elemNum_IncludeFace.at(1)), face_tp)]).pos);
		if (tp1 * tp2 > 0)
		{
			std::cout << "There is two mesh intersection, the meshes number are "
				<< elemNum_IncludeFace.at(0) << ' '
				<< elemNum_IncludeFace.at(1) << '\n';
			exit(-1);
		}
		su_mesh->elem.at(elemNum_IncludeFace.at(0)).neig[Face_Opposite_Node(su_mesh->elem.at(elemNum_IncludeFace.at(0)), face_tp)] = elemNum_IncludeFace.at(1);
		su_mesh->elem.at(elemNum_IncludeFace.at(1)).neig[Face_Opposite_Node(su_mesh->elem.at(elemNum_IncludeFace.at(1)), face_tp)] = elemNum_IncludeFace.at(0);
	}
	else if (elemNum_IncludeFace.size() == 1) // 初始Delaunay三角化四边形边框边
		su_mesh->elem.at(elemNum_IncludeFace.at(0)).neig[Face_Opposite_Node(su_mesh->elem.at(elemNum_IncludeFace.at(0)), face_tp)] = -1;
	return true;
}

void _MESH_PROCESS::Check_Elem_Form_Order(_SU_MESH* su_mesh)
{
	int cnt = 0;
	for (std::vector<ELEM>::iterator iter = su_mesh->elem.begin(); iter != su_mesh->elem.end(); ++iter)
	{
		if ((iter->form[0] < iter->form[1]) && (iter->form[1] < iter->form[2]) && (iter->form[2] < iter->form[3]))
			continue;
		else
			std::cout << "the nodeNum order of elem " << cnt << " is wrong!\n";
		cnt++;
	}
	return;
}

void _MESH_PROCESS::Check_ElemAdjacency_accuracy(_SU_MESH* su_mesh)
{
	if (su_mesh->elem.size() == 0)
		return;
	if (su_mesh->elem.size() != su_mesh->elem_num)
		std::cout << "elem_num is not true!" << std::endl;
	// 检索每个网格单元
	ELEM elem_tp;
	FACE face_tp;
	for (int i = 0; i < su_mesh->elem_num; i++)
	{
		elem_tp = su_mesh->elem.at(i);
		for (int j = 0; j < DIM + 1; j++)
		{
			if (elem_tp.form[j] > su_mesh->node_num)
				std::cout << "the value of the node_num is wrong" << std::endl;
			if (elem_tp.neig[j] != -1)
			{
				if (elem_tp.neig[j] > su_mesh->elem_num)
					std::cout << "the value of the elem_num is wrong" << std::endl;
				else if (!Elem_Adjacent(su_mesh->elem.at(elem_tp.neig[j]), elem_tp))
					std::cout << "The value of the " << i << " elem is wrong" << std::endl;
				face_tp = elem_AdjacentFace(su_mesh->elem.at(elem_tp.neig[j]), elem_tp);
				if (Face_Opposite_Node(elem_tp, face_tp) != j)
					std::cout << "The value of the " << i << " elem is wrong" << std::endl;
				if (su_mesh->elem.at(elem_tp.neig[j]).neig[Face_Opposite_Node(su_mesh->elem.at(elem_tp.neig[j]), face_tp)] != i)
					std::cout << "The value of the " << i << " elem is wrong" << std::endl;
			}
			else
			{
				face_tp = Node_Opposite_Face(elem_tp, elem_tp.form[j]);
				if (face_tp.form[0] >= su_mesh->InitNode_num || face_tp.form[1] >= su_mesh->InitNode_num || face_tp.form[2] >= su_mesh->InitNode_num)
					std::cout << "The value of the " << i << " elem is wrong" << std::endl;
			}
			if (elem_tp.form[j] > su_mesh->node_num)
				std::cout << "the value of the node_num is wrong" << std::endl;
		}
	}
	return;
}

void _MESH_PROCESS::Check_NodeElem_accuracy(_SU_MESH* su_mesh)
{
	if (su_mesh->node.size() != su_mesh->node_num)
		std::cout << "node_num is not true!" << std::endl;
	// 检索每个节点，看每个节点的elem是否准确
	for (int i = 0; i < su_mesh->node_num; i++)
	{
		int elem_Numtp = su_mesh->node.at(i).elem; // 拿出第i个节点的elem值，即第elem_Numtp个网格单元
		if (elem_Numtp >= su_mesh->elem_num)
			std::cout << "the value of the elem_num  is wrong" << std::endl;
		else if (elem_Numtp == -1)
			continue;
		else if (su_mesh->elem.at(elem_Numtp).form[0] == i ||
			su_mesh->elem.at(elem_Numtp).form[1] == i ||
			su_mesh->elem.at(elem_Numtp).form[2] == i ||
			su_mesh->elem.at(elem_Numtp).form[3] == i)
			continue;
		std::cout << "The value of the " << i << " node is wrong" << std::endl;
	}
	return;
}

bool _MESH_PROCESS::Check_Dangling_Node(_SU_MESH* su_mesh)
{
	std::vector<ELEM>::iterator elem_iter;
	for (int cnt = 0; cnt < su_mesh->node_num; cnt++)
	{
		for (elem_iter = su_mesh->elem.begin(); elem_iter < su_mesh->elem.end(); elem_iter++)
			if (elem_iter->form[0] == cnt || elem_iter->form[1] == cnt || elem_iter->form[2] == cnt || elem_iter->form[3] == cnt)
				break;
		if (elem_iter == su_mesh->elem.end())
			return false;
	}
	return true;
}

int _MESH_PROCESS::Check_Dangling_Node(_SU_MESH* su_mesh, int nodeNum_tp, int elemNum_Basis)
{
	std::vector<ELEM>::iterator elem_iter;
	int cnt;
	// 如果elemNum_Basis==-1代表当前程序在插入边界点，cnt从0开始计数，即从第一个节点开始判断悬挂点
	if (elemNum_Basis == -1)
		cnt = 0;
	// 如果当前程序在插入内部点，则从第一个插入的内部点开始判断
	else
		cnt = su_mesh->nodeNum_before_insert_interior_points;
	for (; cnt < nodeNum_tp; cnt++)
	{
		if (std::find(su_mesh->nodeNum_degradation.begin(), su_mesh->nodeNum_degradation.end(), cnt) != su_mesh->nodeNum_degradation.end())
			continue;
		for (elem_iter = su_mesh->elem.begin(); elem_iter < su_mesh->elem.end(); elem_iter++)
			if (elem_iter->form[0] == cnt || elem_iter->form[1] == cnt || elem_iter->form[2] == cnt || elem_iter->form[3] == cnt)
				break;
		if (elem_iter == su_mesh->elem.end())
			return cnt;
	}
	return -1;
}
