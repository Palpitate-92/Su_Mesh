#include "Su_Mesh.h" //引用头文件

void _SU_MESH::check()
{
    // _QUALITY quality;
    // quality.Optimization_based_Smoothing_The_all(this, 379);
    // quality.Optimization_based_Smoothing_The_all(this, 450);
    // std::vector<double> volume;
    // std::vector<double> Shape_Quality;
    // double all_volume = 0;
    // int i = 0;
    // int cnt = 0;
    // for (std::vector<ELEM>::iterator elem_iter = elem.begin(); elem_iter != elem.end(); ++elem_iter)
    // {
    //   volume.push_back(abs(data_process.tetrahedral_volume(
    //       node.at(elem_iter->form[0]),
    //       node.at(elem_iter->form[1]),
    //       node.at(elem_iter->form[2]),
    //       node.at(elem_iter->form[3]))));
    //   all_volume += volume.back();
    //   if (elem_iter->quality < 0.01)
    //   {
    //     cnt++;
    //     // std::cout << i << '\n';
    //   }
    //   Shape_Quality.push_back(elem_iter->quality);
    //   i++;
    // }
    // if (cnt > 0)
    //   std::cout << cnt << ' ';
    // std::sort(volume.begin(), volume.end());
    // std::sort(Shape_Quality.begin(), Shape_Quality.end());
    // if (abs(all_volume - 1000000.0) > 0.1)
    //   std::cout << std::fixed << "Mesh total mass error!\n";
    // std::cout << all_volume << '\n';
    return;
}

int main()
{
    _SU_MESH su_mesh;
    _SHEWCHUK Shewchuk;
    // 为保证Shewchuk的自适应精确计算，首先根据目前设定初始化所有值
    Shewchuk.exactinit();
    // * 读取输入文件
    su_mesh.file_ios.Read_File(&su_mesh, "C:\\Users\\20758\\Desktop\\model\\feiji\\Meshes\\FacemeshBin");
    // 生成初始Delaunay三角化，插入初始Delaunay三角化六面体边框的八个顶角节点，储存并查找边界边长度信息，给所有节点加上密度控制信息
    su_mesh.boundary_point.IniDelaunay(&su_mesh, 0);

    // 输出边界边长度信息文件
    //su_mesh.file_ios.Output_boundary_edge_info(&su_mesh);
    //exit(-1);

    // 将所有节点坐标向上取整
    su_mesh.boundary_point.Rounding(&su_mesh);
    // 输出当前三角化信息文件
    su_mesh.file_ios.Write_File(&su_mesh, "1111\\Ini_Delaunay");
    // * 边界点插入
    su_mesh.boundary_point.Insert_BoundaryPoint(&su_mesh);
    // 输出当前三角化信息文件
    su_mesh.file_ios.Write_File(&su_mesh, "1111\\Boundary_Delaunay");
    // 先移除初始Delaunay三角化的8个顶角节点与包含这些节点的所有网格单元，简化边界恢复步骤
    su_mesh.boundary_recovery.Removal_ExGrid(&su_mesh, 1);
    // 输出当前三角化信息文件
    //su_mesh.file_ios.Write_File(&su_mesh, "Removal_1_Delaunay");
    // * 边界恢复
    su_mesh.boundary_recovery.Recovery_Boundary(&su_mesh);
    // 移除剩余外部单元并缩减容器
    su_mesh.boundary_recovery.Removal_ExGrid(&su_mesh, 2);
    // 输出当前三角化信息文件
    su_mesh.file_ios.Write_File(&su_mesh, "1111\\Removal_Delaunay");
    // 内部点生成与插入
    su_mesh.interior_point.AutoRefine(&su_mesh);
    // 简易判断网格各种信息是否有效
    su_mesh.mesh_process.Check_Elem_Form_Order(&su_mesh);
    su_mesh.mesh_process.Check_ElemAdjacency_accuracy(&su_mesh);
    su_mesh.mesh_process.Check_NodeElem_accuracy(&su_mesh);
    if (!su_mesh.mesh_process.Check_Dangling_Node(&su_mesh))
        std::cout << "There are dangling nodes in the current triangulation!\n";
    // 网格优化前，首先计算各初始网格单元质量
    su_mesh.quality.Calculate_Shape_Quality(&su_mesh);
    // 输出网格质量优化前信息文件 Before_Quality
    // su_mesh.file_ios.Write_File(&su_mesh, "Before_Quality");
    // 输出优化前三角化的网格单元质量信息
    std::cout << "Output information about the quality of mesh elements triangulated before optimization:\n";
    su_mesh.quality.Quality_Information(&su_mesh);
    // 网格质量优化
    for (int i = 0; i < 3; i++)
    {
        // 先利用广义薄元分解实现网格质量优化
        su_mesh.quality.Quality_Optimization_SliverRemoval(&su_mesh);
        // 后利用节点光顺实现网格质量优化
        su_mesh.quality.Quality_Optimization_Smoothing(&su_mesh);
        // 再利用面交换实现网格质量优化
        su_mesh.quality.Quality_Optimization_Face_Transform(&su_mesh);
        su_mesh.counter++;
    }
    // 输出优化后三角化的网格单元质量信息
    std::cout << "Output optimized triangulated mesh element quality information:\n";
    su_mesh.quality.Quality_Information(&su_mesh);
    // 简易判断网格各种信息是否有效
    su_mesh.check();
    su_mesh.mesh_process.Check_ElemAdjacency_accuracy(&su_mesh);
    su_mesh.mesh_process.Check_NodeElem_accuracy(&su_mesh);
    if (!su_mesh.mesh_process.Check_Dangling_Node(&su_mesh))
        std::cout << "There are dangling nodes in the current triangulation!\n";
    // 输出网格质量优化后信息文件 Final_Delaunay
    su_mesh.file_ios.Write_File(&su_mesh, "1111\\Final_Delaunay");
    return 0;
    // #include <ctime>
    // clock_t start, end;
    // start = clock();
    // end = clock();
    // double elapsedTime = static_cast<double>(end - start) / CLOCKS_PER_SEC;
    // // clock()以毫秒的形式展现，因此需要除以 CLOCKS_PER_SEC 来实现转换
    // // static_cast<double>的作用是将结果转换为double类型
    // printf("CPU PROCESSING TIME: %f", elapsedTime);
}

/**
 *  1.边界恢复
 *  2.完善空腔修复算法
 * ! 3.完善网格节点NODE类中spac的使用
 *  4.优化各种查找算法
 * ! 5.质量优化
 * * for (std::vector<ELEM>::iterator elem_iter = elem->begin(); elem_iter !=elem->end();++elem_iter)
 * * std::vector<NODE>().swap(node_Insert);               //
 * 初始化node_Insert，并释放容器空间
 * * if (std::find(elemNum_succ.begin(), elemNum_succ.end(), elem_tp.neig[i])
 * == elemNum_succ.end())
 * * i=std::distance(elemNum_succ.begin(),iter)
 * * std::vector<NODE>().swap(node_Insert);               //
 * 初始化node_Insert，并释放容器空间
 */

/*
  1.光标选中想要注释的所有代码，ctrl+/，取消同理。
  2.光标选中想要注释的所有代码，alt+shift+a，取消同理。
  3.光标选中想要注释的所有代码，(1）ctrl+k（2）ctrl+c，取消是（1）ctrl+k（2）ctrl+u。
  4.迭代器和下标之间转换主要是使用stl中的advance和distance函数来进行的，advance是将iterator移动指定个元素，distance是计算两个iterator直接的距离。
  5.折叠所有代码 先ctrl+m 再ctrl+o(这是字母O) 2.展开所有代码 先ctrl+m 再ctrl+l(这是字母L)
  */