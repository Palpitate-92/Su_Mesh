#include "Su_Mesh.h"

void _IOS::Read_File(_SU_MESH* su_mesh, std::string infileName)
{
	std::fstream infile;                                        // 声明输入文件流
	infile.open(infileName, std::ios::in | std::ios::binary);   // 打开infileName，并且只能进行读取操作
	std::cout << "Reading from the file......" << std::endl;    // 正在读取文件
	if (!infile.is_open())
	{                                                           // 判断文件是否存在或者是否成功打开
		std::cout << "error on open " << infileName << std::endl; // 文件不能成功打开
		exit(-1);
	}
	else
	{
		std::cout << "Reading successfully!" << std::endl; // 读取文件成功
		// 读入节点信息
		int tp[2];
		infile.read((char*)&tp, sizeof(int) * 2); // 过滤无关信息
		infile.read((char*)&su_mesh->node_num, sizeof(int));
		su_mesh->InitNode_num = su_mesh->node_num; // 储存初始节点数目
		NODE node_tp;                              // 声明临时NODE类用于压入容器
		for (int i = 0; i < su_mesh->node_num; i++)
		{
			infile.read((char*)node_tp.pos, sizeof(double) * 3);
			su_mesh->node.push_back(node_tp); // 压入容器node
		}
		// 读入表面网格信息
		infile.read((char*)&su_mesh->boundaryFace_num, sizeof(int));
		FACE face_tp;
		for (int i = 0; i < su_mesh->boundaryFace_num; i++)
		{
			infile.read((char*)face_tp.form, sizeof(int) * 3);
			face_tp.Sort();
			su_mesh->boundary_face.push_back(face_tp); // 压入容器
		}
	}
	infile.close();
	return;
}

void _IOS::Write_File(_SU_MESH* su_mesh, std::string outfileName)
{
	// 输出节点信息文件*.node
	std::fstream outfile_node; // 声明输出文件流
	std::string filePath_node = "MESH\\" + outfileName + ".node";
	outfile_node.open(filePath_node, std::ios::out | std::ios::trunc);                        // 创建或者打开输出文件，如果此文件已经存在, 则打开文件之前把文件长度截断为0
	if (!outfile_node.is_open())
	{                                                              // 判断文件是是否成功打开
		std::cout << "error on open " << filePath_node << std::endl; // 文件不能成功打开
		system("pause");
		exit(-1);
	}
	else
	{
		outfile_node << su_mesh->node_num << ' ' << DIM << " 0 0\n";
		int node_cnt = 0; // 定义节点输出标号变量，第一个节点标号显然为零
		// 使用迭代器遍历node，输出节点信息
		for (std::vector<NODE>::iterator node_iter = su_mesh->node.begin(); node_iter != su_mesh->node.end();
			++node_iter)
		{
			outfile_node << node_cnt << ' ' << node_iter->pos[0] << ' ' << node_iter->pos[1] << ' ' << node_iter->pos[2]
				<< '\n';
			node_cnt += 1; // 每成功输出一个节点，node_cnt++
		}
	}
	outfile_node.close();
	// 输出四面体网格信息文件*.ele
	std::fstream outfile_ele; // 声明输出文件流
	std::string filePath_ele = "MESH\\" + outfileName + ".ele";
	outfile_ele.open(filePath_ele, std::ios::out | std::ios::trunc);                        // 创建或者打开输出文件，如果此文件已经存在, 则打开文件之前把文件长度截断为0
	if (!outfile_ele.is_open())
	{                                                             // 判断文件是是否成功打开
		std::cout << "error on open " << filePath_ele << std::endl; // 文件不能成功打开
		system("pause");
		exit(-1);
	}
	else
	{
		outfile_ele << su_mesh->elem_num << " 4 0\n";
		int ele_cnt = 1; // 定义网格单元输出标号变量，第一个网格单元标号显然为1
		// 使用迭代器遍历elem，输出网格单元信息
		for (std::vector<ELEM>::iterator elem_iter = su_mesh->elem.begin(); elem_iter != su_mesh->elem.end();
			++elem_iter)
			outfile_ele << ele_cnt++ << ' ' << elem_iter->form[0] << ' ' << elem_iter->form[1] << ' '
			<< elem_iter->form[2] << ' ' << elem_iter->form[3] << '\n';
	}
	outfile_ele.close();
	std::cout << outfileName << " outing successfully !" << std::endl; // 文件输出成功
	return;
}

void _IOS::Write_File_judge(_SU_MESH* su_mesh, std::string outfileName)
{                                                                    // 输出节点信息文件*.node
	std::fstream outfile_node;                                         // 声明输出文件流
	std::string filePath_node = "MESH\\" + outfileName + ".node";
	outfile_node.open(filePath_node, std::ios::out | std::ios::trunc); // 创建或者打开输出文件，如果此文件已经存在, 则打开文件之前把文件长度截断为0
	if (!outfile_node.is_open())
	{                                                                  // 判断文件是是否成功打开
		std::cout << "error on open " << filePath_node << std::endl;     // 文件不能成功打开
		system("pause");
		exit(-1);
	}
	else
	{
		outfile_node << su_mesh->node_num << ' ' << DIM << " 0 0\n";
		int node_cnt = 0; // 定义节点输出标号变量，第一个节点标号显然为零
		// 使用迭代器遍历node，输出节点信息
		for (std::vector<NODE>::iterator node_iter = su_mesh->node.begin(); node_iter != su_mesh->node.end();
			++node_iter)
		{
			outfile_node << node_cnt << ' ' << node_iter->pos[0] << ' ' << node_iter->pos[1] << ' ' << node_iter->pos[2]
				<< '\n';
			node_cnt += 1; // 每成功输出一个节点，node_cnt++
		}
	}
	outfile_node.close();
	// 输出四面体网格信息文件*.ele
	std::fstream outfile_ele;                                        // 声明输出文件流
	std::string filePath_ele = "MESH\\" + outfileName + ".ele";
	outfile_ele.open(filePath_ele, std::ios::out | std::ios::trunc); // 创建或者打开输出文件，如果此文件已经存在, 则打开文件之前把文件长度截断为0
	if (!outfile_ele.is_open())
	{                                                                // 判断文件是是否成功打开
		std::cout << "error on open " << filePath_ele << std::endl;    // 文件不能成功打开
		system("pause");
		exit(-1);
	}
	else
	{
		int cnt = 0;
		for (std::vector<ELEM>::iterator elem_iter = su_mesh->elem.begin(); elem_iter != su_mesh->elem.end();
			++elem_iter)
			if (elem_iter->form[0] != -1)
				cnt++;
		outfile_ele << cnt << " 4 0\n";
		int ele_cnt = 1; // 定义网格单元输出标号变量，第一个网格单元标号显然为1
		// 使用迭代器遍历elem，输出网格单元信息
		for (std::vector<ELEM>::iterator elem_iter = su_mesh->elem.begin(); elem_iter != su_mesh->elem.end();
			++elem_iter)
			if (elem_iter->form[0] == -1)
				continue;
			else
				outfile_ele << ele_cnt++ << ' ' << elem_iter->form[0] << ' ' << elem_iter->form[1] << ' '
				<< elem_iter->form[2] << ' ' << elem_iter->form[3] << '\n';
	}
	outfile_ele.close();
	std::cout << outfileName << " outing successfully !" << std::endl; // 文件输出成功
	return;
}


void _IOS::Output_boundary_edge_info(_SU_MESH* su_mesh)
{
	_DATA_PROCESS data_process;
	std::fstream outfile; // 声明输出文件流
	std::string filePath = "boundary_edge.txt";
	outfile.open(filePath, std::ios::out | std::ios::trunc);                        // 创建或者打开输出文件，如果此文件已经存在, 则打开文件之前把文件长度截断为0
	if (!outfile.is_open())
	{                                                              // 判断文件是是否成功打开
		std::cout << "error on open " << filePath << std::endl; // 文件不能成功打开
		system("pause");
		exit(-1);
	}
	else
	{
		for (std::vector<EDGE>::iterator edge_iter = su_mesh->boundary_edge.begin(); edge_iter != su_mesh->boundary_edge.end(); ++edge_iter)
			outfile << data_process.get_dist(su_mesh->node.at(edge_iter->form[0]).pos, su_mesh->node.at(edge_iter->form[1]).pos) << '\n';
	}
	outfile.close();
	std::string filePath_node = "node.txt";
	outfile.open(filePath_node, std::ios::out | std::ios::trunc);                        // 创建或者打开输出文件，如果此文件已经存在, 则打开文件之前把文件长度截断为0
	if (!outfile.is_open())
	{                                                              // 判断文件是是否成功打开
		std::cout << "error on open " << filePath_node << std::endl; // 文件不能成功打开
		system("pause");
		exit(-1);
	}
	else
	{
		for (std::vector<NODE>::iterator node_iter = su_mesh->node.begin(); node_iter != su_mesh->node.end(); ++node_iter)
			outfile << node_iter->spac << '\n';
	}
	outfile.close();
	return;
}