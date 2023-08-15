#include "Su_Mesh.h" //引用头文件

void _SU_MESH::check()
{
    int a;
    // 首先定义CGAL的顶点类
    Point_3 p1{0, 0, 0},
        p2{0, 1, 0},
        p3{1, 0, 0},
        p4{0, 0, 0},
        p5{0, 1, 0},
        p6{0, 0, 1},
        p7{1, 0, 1};
    // 再定义CGAL的面类
    Triangle_3 tri(p1, p2, p3);
    // 定义CGAL的四面体类
    Tetrahedron_3 tet1(p4, p5, p6, p7);
    // 计算相交结果
    const auto result = intersection(tri, tet1);
    // 如果相交
    if (result)
    {
        if (boost::get<Point_3>(&*result))
            a = 1;
        else if (boost::get<Segment_3>(&*result))
            a = 2;
        else if (boost::get<Triangle_3>(&*result))
            a = 3;
        else if (boost::get<std::vector<CGAL::Point_3<CGAL::Epick>, std::allocator<CGAL::Point_3<CGAL::Epick>>>>(&*result))
            a = 4;
        else
            a = 0;
    }
    else
    {
        std::cout << "Edge-to-elem intersection judgment error!\n";
        system("pause");
    }
    return;
}

int main()
{
    clock_t start, end;
    start = clock();

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
    //system("pause");

    // 将所有节点坐标向上取整
    //su_mesh.boundary_point.Rounding(&su_mesh);
    // 输出当前三角化信息文件
    su_mesh.file_ios.Write_File(&su_mesh, "1111\\Ini_Delaunay");
    // * 边界点插入
    su_mesh.boundary_point.Insert_BoundaryPoint(&su_mesh);
    // 输出当前三角化信息文件
    su_mesh.file_ios.Write_File(&su_mesh, "1111\\Boundary_Delaunay");
    // 标记所有外部网格单元

    // 先移除初始Delaunay三角化的8个顶角节点与包含这些节点的所有网格单元，简化边界恢复步骤
    //su_mesh.boundary_recovery.Removal_ExGrid(&su_mesh, 1);
    // 输出当前三角化信息文件
    //su_mesh.file_ios.Write_File(&su_mesh, "1111\\Removal_1_Delaunay");
    // * 边界恢复
    su_mesh.boundary_recovery.Recovery_Boundary(&su_mesh);

    // 简易判断网格各种信息是否有效
    su_mesh.mesh_process.Judge_the_validity_of_information(&su_mesh);

    end = clock();
    double elapsedTime = static_cast<double>(end - start) / CLOCKS_PER_SEC;
    // clock()以毫秒的形式展现，因此需要除以 CLOCKS_PER_SEC 来实现转换
    // static_cast<double>的作用是将结果转换为double类型
    printf("CPU PROCESSING TIME: %f", elapsedTime);

    // 移除剩余外部单元并缩减容器
    su_mesh.boundary_recovery.Removal_ExGrid(&su_mesh, 2);

    // 简易判断网格各种信息是否有效
    su_mesh.mesh_process.Judge_the_validity_of_information(&su_mesh);

    // 输出当前三角化信息文件
    su_mesh.file_ios.Write_File(&su_mesh, "1111\\Removal_Delaunay");
    // 内部点生成与插入
    su_mesh.interior_point.AutoRefine(&su_mesh);
    // 简易判断网格各种信息是否有效
    su_mesh.mesh_process.Judge_the_validity_of_information(&su_mesh);
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
    su_mesh.mesh_process.Judge_the_validity_of_information(&su_mesh);
    // 输出网格质量优化后信息文件 Final_Delaunay
    su_mesh.file_ios.Write_File(&su_mesh, "1111\\Final_Delaunay");

    // #include <ctime>
    // clock_t start, end;
    // start = clock();
    //end = clock();
    //double elapsedTime = static_cast<double>(end - start) / CLOCKS_PER_SEC;
    // clock()以毫秒的形式展现，因此需要除以 CLOCKS_PER_SEC 来实现转换
    // static_cast<double>的作用是将结果转换为double类型
    //printf("CPU PROCESSING TIME: %f", elapsedTime);

    return 0;
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
 * * if (std::find(elemNum_succ.begin(), elemNum_succ.end(), elem_tp.neig[i])== elemNum_succ.end())
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