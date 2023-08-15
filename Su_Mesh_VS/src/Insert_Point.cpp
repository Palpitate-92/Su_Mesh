#include "Su_Mesh.h"

int _INSERT_POINT::Insert_point(_SU_MESH *su_mesh, int nodeNum_Last_succ, int nodeNum_Insert, NODE node_Insert, int elemNum_Basis)
{
    _CAVITY cavity;
    _SHEWCHUK shewchuk;
    _DATA_PROCESS data_process;
    _MESH_PROCESS mesh_process;
    std::vector<int> elemNum_Cavity; // 创建一个空腔容器，用来储存外接球包含待插入节点node_Insert的网格单元编号
    // 若elemNum_Basis==-1代表当前是在插入边界点，否则则是插入内部点
    // 如果是在插入边界点，则首先查找基单元
    if (elemNum_Basis == -1)
    {
        int elemNum_tp = mesh_process.Find_Elem_DirectIncludeNode(su_mesh, nodeNum_Last_succ == -1 ? -1 : su_mesh->node.at(nodeNum_Last_succ).elem, node_Insert);
        if (elemNum_tp == -1)
            std::cout << "Boundary point lookup failed, in_tetrahedron run error, and the number of the current inserted boundary point is " << nodeNum_Insert << std::endl, system("pause");
        elemNum_Cavity.push_back(elemNum_tp);
    }
    // 如果是在插入内部点，则直接压入基单元编号到elemNum_Cavity
    else
        elemNum_Cavity.push_back(elemNum_Basis);
    if (!cavity.Find_Cavity(su_mesh, &elemNum_Cavity, node_Insert, elemNum_Cavity.front()))
        return 2;                          // 空腔查找
    std::vector<FACE> face_Cavity;         // 创建一个容器，用来储存空腔边界面
    std::vector<FACE>::iterator face_iter; // 创建一个迭代器，便于删除非空腔边界面
    ELEM elem_tp;
    FACE face_tp;
    /**
	 * * 在face_Cavity内查找当前搜索的网格单元边界面，如果搜到end()，代表没有找到，该边是空腔边界面，可以插入；
	 * * 如果没有搜到end()，代表该面不是空腔边界面，不予插入，并且在face_Cavity内删除该网格面
	 */
    // 遍历elemNum_Cavity容器，将所有空腔边界面压入face_Cavity
    for (std::vector<int>::iterator iter = elemNum_Cavity.begin(); iter != elemNum_Cavity.end(); ++iter)
    // 一个网格单元有四个面
    {
        for (int i = 0; i < DIM + 1; i++)
        {
            face_tp = mesh_process.Node_Opposite_Face(su_mesh->elem.at(*iter), su_mesh->elem.at(*iter).form[i]);
            // 修改face_tp的form顺序，使其正方向排序
            if (shewchuk.orient3d(su_mesh->node.at(face_tp.form[0]).pos, su_mesh->node.at(face_tp.form[1]).pos, su_mesh->node.at(face_tp.form[2]).pos, su_mesh->node.at(su_mesh->elem.at(*iter).form[i]).pos) < 0)
                std::swap(face_tp.form[0], face_tp.form[1]);
            // 在face_Cavity容器内查找当前判断网格面
            if ((face_iter = std::find(face_Cavity.begin(), face_Cavity.end(), face_tp)) == face_Cavity.end())
            {
                // 显然，当前判断网格面不存在于face_Cavity中，进行下一步操作
                // 首先判断elemNum_Basis值，若是-1则代表当前是在插入边界点，不需要判断密度信息，直接插入当前判断网格面到face_Cavity中
                if (elemNum_Basis == -1)
                    face_Cavity.push_back(face_tp);
                else
                {
                    // 根据网格单元边理想长度判断该node_Insert是否能插入
                    if ((data_process.get_dist(node_Insert.pos, su_mesh->node.at(face_tp.form[0]).pos) >= node_Insert.spac) &&
                        (data_process.get_dist(node_Insert.pos, su_mesh->node.at(face_tp.form[1]).pos) >= node_Insert.spac) &&
                        (data_process.get_dist(node_Insert.pos, su_mesh->node.at(face_tp.form[2]).pos) >= node_Insert.spac))
                        face_Cavity.push_back(face_tp);
                    else // 不符合网格单元边理想长度，不予插入该节点，返回3
                        return 3;
                }
            }
            else
                face_Cavity.erase(face_iter); // 显然，该网格面被插入过face_Cavity，说明该边不是空腔边界面，删除该面
        }
    }
    cavity.RepairCavity(su_mesh, node_Insert, &elemNum_Cavity, &face_Cavity); // 空腔修复
    if (elemNum_Cavity.empty())
        return 2;
    // 运行到这里，并且elemNum_Basis!=-1即当前程序是在插入内部点的情况下，说明当前待插入点满足密度控制信息
    // 首先将当前待插入点插入su_mesh->node容器内，形成节点插入
    if (elemNum_Basis != -1)
    {
        su_mesh->node.push_back(node_Insert);
        su_mesh->node_num += 1;
    }
    // 空腔初始化操作，在elem内中初始化空腔所包含的网格单元
    cavity.Initialize_Cavity(su_mesh, elemNum_Cavity);
    // 判断空腔初始化后是否存在悬挂点
    int dangling_judge = mesh_process.Check_Dangling_Node(su_mesh, nodeNum_Insert, elemNum_Basis);
    if (dangling_judge != -1)
    {
        std::cout << "The node " << dangling_judge << " is dangling!\n";
        system("pause");
    }
    // 对每个空腔边界面进行操作，生成待插入点与当前空腔边界面形成的网格单元，首先更新空腔边界面的相邻关系
    // 首先将elemNum_Cavity容器内全部元素压入su_mesh->elemNum_invalid
    for (std::vector<int>::iterator iter = elemNum_Cavity.begin(); iter != elemNum_Cavity.end(); iter++)
        su_mesh->elemNum_invalid.push_back(*iter);
    // std::sort(su_mesh->elemNum_invalid.begin(), su_mesh->elemNum_invalid.end());
    int elemNum_tp = 0;
    for (std::vector<FACE>::iterator iter = face_Cavity.begin(); iter != face_Cavity.end(); ++iter)
    {
        // 首先使用su_mesh->elemNum_invalid容器内网格单元编号
        if (!su_mesh->elemNum_invalid.empty())
        {
            elemNum_tp = su_mesh->elemNum_invalid.front(); // 取出su_mesh->elemNum_invalid内一个网格单元编号
            su_mesh->elemNum_invalid.erase(su_mesh->elemNum_invalid.begin());
            // 显然可以直接给新生成的网格单元赋节点编号
            su_mesh->elem.at(elemNum_tp).form[0] = iter->form[0];
            su_mesh->elem.at(elemNum_tp).form[1] = iter->form[1];
            su_mesh->elem.at(elemNum_tp).form[2] = iter->form[2];
            su_mesh->elem.at(elemNum_tp).form[3] = nodeNum_Insert;
            // 交换网格单元节点信息form数组，使其从小到大排列
            su_mesh->elem.at(elemNum_tp).Sort();
            // 改变当前网格单元所有节点的elem值
            su_mesh->node.at(iter->form[0]).elem = elemNum_tp;
            su_mesh->node.at(iter->form[1]).elem = elemNum_tp;
            su_mesh->node.at(iter->form[2]).elem = elemNum_tp;
            su_mesh->node.at(nodeNum_Insert).elem = elemNum_tp;
        }
        // 之后生成的网格全压入elem中，并赋予新的编号
        else
        {
            // 显然可以直接给新生成的网格单元赋节点编号
            elem_tp.form[0] = iter->form[0];
            elem_tp.form[1] = iter->form[1];
            elem_tp.form[2] = iter->form[2];
            elem_tp.form[3] = nodeNum_Insert;
            // 交换网格单元节点信息form数组，使其从小到大排列
            elem_tp.Sort();
            // 改变当前网格单元所有节点的elem值
            su_mesh->node.at(iter->form[0]).elem = su_mesh->elem_num;
            su_mesh->node.at(iter->form[1]).elem = su_mesh->elem_num;
            su_mesh->node.at(iter->form[2]).elem = su_mesh->elem_num;
            su_mesh->node.at(nodeNum_Insert).elem = elemNum_tp;
            su_mesh->elem.push_back(elem_tp); // 将elem_tp压入elem中，形成网格单元的生成
            su_mesh->elem_num += 1;
        }
        // 查找包含当前判断边界面的所有网格单元，并更新相邻关系
        if (!mesh_process.Update_Djacency(su_mesh, *iter, "slow"))
        {
            std::cout << nodeNum_Insert << " Failed to update djacency\n";
            system("pause");
        }
    }
    std::vector<EDGE> edge_Cavity;         // 创建一个容器，用来储存所有空腔边界边
    std::vector<EDGE>::iterator edge_iter; // 创建一个迭代器，便于删除非空腔边界边
    EDGE edge_tp;
    // 遍历face_Cavity容器，将所有空腔边界边压入edge_Cavity
    for (std::vector<FACE>::iterator iter = face_Cavity.begin(); iter != face_Cavity.end(); ++iter)
        // 一个网格面有三条边
        for (int i = 0; i < DIM; i++)
        {
            edge_tp = mesh_process.Node_Opposite_Edge(*iter, iter->form[i]);
            // 在face_Cavity容器内查找当前判断网格面
            if ((edge_iter = std::find(edge_Cavity.begin(), edge_Cavity.end(), edge_tp)) == edge_Cavity.end())
                // 显然，当前判断空腔边界边不存在于edge_Cavity中，则压入edge_Cavity
                edge_Cavity.push_back(edge_tp);
        }
    // 更新插入点与所有空腔边界边连接形成的网格面的相邻关系
    for (std::vector<EDGE>::iterator iter = edge_Cavity.begin(); iter != edge_Cavity.end(); ++iter)
    {
        face_tp.form[0] = iter->form[0];
        face_tp.form[1] = iter->form[1];
        face_tp.form[2] = nodeNum_Insert;
        face_tp.Sort();
        if (!mesh_process.Update_Djacency(su_mesh, face_tp, "slow"))
        {
            std::cout << nodeNum_Insert << " Failed to update djacency\n";
            system("pause");
        }
    }
    return 1;
}
