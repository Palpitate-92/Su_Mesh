/*
 * 声明输入输出算法类
 */

 // #pragma once
#ifndef _IOS_H
#define _IOS_H

#include <string>
#include "data.h"

// 声明文件流输入输出类
class _IOS
{
public:
	void Read_File(_SU_MESH* su_mesh, std::string infileName);   // 从文件内读取网格信息
	void Write_File(_SU_MESH* su_mesh, std::string outfileName); // 输出网格信息到文件
	void Write_File_judge(_SU_MESH* su_mesh, std::string outfileName);

	// 输出边界边长度信息文件
	void Output_boundary_edge_info(_SU_MESH* su_mesh);

private:
};

#endif