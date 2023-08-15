#include "Su_Mesh.h"

void _INTERIOR_POINT::CreateFieldNodes(_SU_MESH *su_mesh, std::vector<NODE> *node_Insert, std::vector<int> *elemNum_Basis_Initial)
{
    NODE node_tp; // 声明一个临时储存待插入节点的NODE类
    int cnt = 0;  // 定义一个变量，来对网格编号进行计数
    _DATA_PROCESS data_process;
    for (std::vector<ELEM>::iterator elem_iter = su_mesh->elem.begin(); elem_iter != su_mesh->elem.end(); ++elem_iter)
    {
        // if (elem_iter->DeFrame_Grid) // 如果该单元是初始Delaunay三角化方形边框的四个顶角节点与边界点形成的单元，则跳过
        // {
        //   cnt++;
        //   continue;
        // }
        // 计算当前网格单元重心
        if (elem_iter->form[0] == -1)
            continue;
        node_tp = data_process.center_of_gravity(su_mesh->node.at(elem_iter->form[0]), su_mesh->node.at(elem_iter->form[1]), su_mesh->node.at(elem_iter->form[2]), su_mesh->node.at(elem_iter->form[3]));
        // ! 由于当前三角化每个节点密度信息相同并且没有额外密度控制信息，所以此处直接给节点spac赋值
        //node_tp.spac = su_mesh->node.at(elem_iter->form[0]).spac;
        // 此处新节点的密度控制信息用当前网格单元四个节点密度控制信息的均值
        node_tp.spac = (su_mesh->node.at(elem_iter->form[0]).spac + su_mesh->node.at(elem_iter->form[1]).spac + su_mesh->node.at(elem_iter->form[2]).spac + su_mesh->node.at(elem_iter->form[3]).spac) / 4.0;
        if ((data_process.get_dist(node_tp.pos, su_mesh->node.at(elem_iter->form[0]).pos) >= node_tp.spac) &&
            (data_process.get_dist(node_tp.pos, su_mesh->node.at(elem_iter->form[1]).pos) >= node_tp.spac) &&
            (data_process.get_dist(node_tp.pos, su_mesh->node.at(elem_iter->form[2]).pos) >= node_tp.spac) &&
            (data_process.get_dist(node_tp.pos, su_mesh->node.at(elem_iter->form[3]).pos) >= node_tp.spac))
        {
            node_Insert->push_back(node_tp);       // 将生成的节点插入node_Insert
            elemNum_Basis_Initial->push_back(cnt); // 将包含该生成节点的网格单元编号插入elemNum_Basis_Initial
        }
        cnt++;
    }
    return;
}

// 内部点生成和插入
void _INTERIOR_POINT::AutoRefine(_SU_MESH *su_mesh)
{
    su_mesh->nodeNum_before_insert_interior_points = su_mesh->node_num; // 储存插入内部点前节点数
    std::vector<NODE> node_Insert;                                      // 创建储存待插入节点的容器
    // 创建一个容器，用来储存生成插入点时初始基单元编号，标号与node_Insert一一对应
    // 该容器作为后续网格变化后再次查找包含某插入节点的网格提供便利，能优化查找时间
    std::vector<int> elemNum_Basis_Initial;
    int cnt = 0;                      // 计数器
    int judgment;                     // 判断节点是否成功插入
    bool node_insert_judgment_first;  // 判断第一层while循环是否有节点被成功插入
    bool node_insert_judgment_second; // 判断第二层while循环是否有节点被成功插入
    _MESH_PROCESS mesh_process;
    _INSERT_POINT insert_point;
    do
    {
        std::vector<NODE>().swap(node_Insert);          // 初始化node_Insert，并释放容器空间
        std::vector<int>().swap(elemNum_Basis_Initial); // 初始化elemNum_Basis_Initial，并释放容器空间
        node_insert_judgment_first = false;             // 重置信号
        // 在当前三角化所有网格重心处批量生成内部点，并在生成内部点时检查密度控制信息，不满足密度信息的内部点不予生成
        CreateFieldNodes(su_mesh, &node_Insert, &elemNum_Basis_Initial);
        while (!node_Insert.empty()) // 如果待插入节点容器不为空，则开始插入节点
        {
            cnt = 0;
            node_insert_judgment_second = false;
            for (std::vector<NODE>::iterator node_iter = node_Insert.begin(); node_iter != node_Insert.end();)
            {
                // 声明一个变量，用来储存一个直接包含待插入节点的网格单元编号，作为基单元来查找空腔
                int elemNum_Basis;
                elemNum_Basis = mesh_process.Find_Elem_DirectIncludeNode(su_mesh, elemNum_Basis_Initial.at(cnt), *node_iter);
                // 如果该值为-1，基单元查找失败
                if (elemNum_Basis == -1)
                    std::cout << "When insert interior point, base unit lookup failed, in_tetrahedron run error!\n", system("pause");
                //// 如果该值为-2，代表出现5点共球情况，延迟插入该节点
                //else if (elemNum_Basis == -2)
                //{
                //    ++node_iter;
                //    ++cnt;
                //    continue;
                //}
                // 插入该节点，并得到节点是否成功插入的信息
                judgment = insert_point.Insert_point(su_mesh, su_mesh->node_num - 1, su_mesh->node_num, *node_iter, elemNum_Basis); // 节点插入成功
                if (judgment == 1)
                {
                    node_iter = node_Insert.erase(node_iter);
                    elemNum_Basis_Initial.erase(elemNum_Basis_Initial.begin() + cnt);
                    node_insert_judgment_first = true;
                    node_insert_judgment_second = true;
                }
                // 出现5点共球，延迟插入该节点
                else if (judgment == 2)
                {
                    ++node_iter;
                    ++cnt;
                }
                // 节点插入失败，不满足密度控制信息，直接删除该节点
                else if (judgment == 3)
                {
                    node_iter = node_Insert.erase(node_iter);
                    elemNum_Basis_Initial.erase(elemNum_Basis_Initial.begin() + cnt);
                }
            }
            if (!node_insert_judgment_second)
                break;
        }
        if (!node_Insert.empty())
        {
            std::cout << "There are " << node_Insert.size() << " nodes that were not successfully inserted!\n";
        }
    } while (node_insert_judgment_first); // 如果该次循环成功插入了节点，则再执行一次循环
    return;
}
