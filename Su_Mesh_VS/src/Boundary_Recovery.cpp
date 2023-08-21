#include "Su_Mesh.h"

void _BOUNDARY_RECOVERY::External_Elem_Lookup(_SU_MESH *su_mesh, std::vector<int> *nodeNum_Remove, std::vector<int> *elemNum_Remove)
{
    _MESH_PROCESS mesh_process;
    // 采用着色法来确定外部单元，若两个网格单元相邻网格面为边界面，则其标记相反
    int *external_elem_judge = (int *)malloc((su_mesh->elem_num) * sizeof(int)); // 作为每个网格单元的标记数组
    memset(external_elem_judge, 0, (su_mesh->elem_num) * sizeof(int));           // 1代表非外部单元，-1代表外部单元,0代表未被判断
    int ini_elemNum = 0;
    // 该方法最重要的是确定初始单元及其内外部信息，此处直接从包含初始Delaunay三角化边框节点的网格单元入手，该网格单元一定是外部网格单元
    ini_elemNum = su_mesh->node.at(su_mesh->Delaunay_Frame_numPos[0]).elem;
    external_elem_judge[ini_elemNum] = -1;
    std::deque<int> elemNum_wait;
    std::vector<int> elemNum_succ;
    elemNum_wait.push_back(ini_elemNum);
    elemNum_succ.push_back(ini_elemNum);
    ELEM elem_tp;
    FACE face_tp;
    int elemNum_tp;
    int elemNum_tp_neig;
    while (int(elemNum_succ.size()) < su_mesh->elem_num)
    {
        elemNum_tp = elemNum_wait.front();
        elem_tp = su_mesh->elem.at(elemNum_tp);
        elemNum_wait.pop_front();
        for (int i = 0; i < DIM + 1; i++)
        {
            elemNum_tp_neig = elem_tp.neig[i];
            if (elemNum_tp_neig == -1)
                continue;
            if (std::find(elemNum_succ.begin(), elemNum_succ.end(), elemNum_tp_neig) != elemNum_succ.end())
                continue;
            face_tp = mesh_process.Node_Opposite_Face(elem_tp, elem_tp.form[i]);
            if (std::find(su_mesh->boundary_face.begin(), su_mesh->boundary_face.end(), face_tp) == su_mesh->boundary_face.end())
                external_elem_judge[elemNum_tp_neig] = external_elem_judge[elemNum_tp];
            else
                external_elem_judge[elemNum_tp_neig] = -external_elem_judge[elemNum_tp];
            elemNum_wait.push_back(elemNum_tp_neig);
            elemNum_succ.push_back(elemNum_tp_neig);
        }
        if (external_elem_judge[elemNum_tp] == -1)
            for (int j = 0; j < 4; j++)
                if (elem_tp.form[j] >= su_mesh->InitNode_num) // 保证该点不是初始边界点
                    if (std::find(nodeNum_Remove->begin(), nodeNum_Remove->end(), elem_tp.form[j]) == nodeNum_Remove->end())
                        nodeNum_Remove->push_back(elem_tp.form[j]);
    }
    for (int i = 0; i < su_mesh->elem_num; i++)
    {
        if (external_elem_judge[i] == -1)
            elemNum_Remove->push_back(i);
        if (external_elem_judge[i] == 0)
            std::cout << i << " elem check error, External_Elem_Lookup fail !\n";
    }
    free(external_elem_judge);
    external_elem_judge = nullptr;
    return;
}

void _BOUNDARY_RECOVERY::ReplaceNode_two(_SU_MESH *su_mesh, int nodeNum_one, int nodeNum_two)
{
    std::vector<int> elemNum_NodeOne; // 创建一个容器，用来储存包含第一个节点的所有网格单元编号
    std::vector<int> elemNum_NodeTwo; // 创建一个容器，用来储存包含第二个节点的所有网格单元编号
    _MESH_PROCESS mesh_process;
    mesh_process.FindBall_fast(su_mesh, nodeNum_one, &elemNum_NodeOne); // 查找包含第一个节点的所有网格单元编号
    mesh_process.FindBall_fast(su_mesh, nodeNum_two, &elemNum_NodeTwo); // 查找包含第二个节点的所有网格单元编号
    // 修改elem容器内值
    for (std::vector<int>::iterator iter = elemNum_NodeOne.begin(); iter != elemNum_NodeOne.end(); ++iter)
    {
        su_mesh->elem.at(*iter).form[mesh_process.Elem_Include_Node(su_mesh->elem.at(*iter), nodeNum_one)] = nodeNum_two;
    }
    for (std::vector<int>::iterator iter = elemNum_NodeTwo.begin(); iter != elemNum_NodeTwo.end(); ++iter)
    {
        su_mesh->elem.at(*iter).form[mesh_process.Elem_Include_Node(su_mesh->elem.at(*iter), nodeNum_two)] = nodeNum_one;
    }
    // 替换节点位置
    NODE node_tp = su_mesh->node.at(nodeNum_one);
    su_mesh->node.at(nodeNum_one) = su_mesh->node.at(nodeNum_two);
    su_mesh->node.at(nodeNum_two) = node_tp;
    return;
}

void _BOUNDARY_RECOVERY::ReplaceElem_two(_SU_MESH *su_mesh, int elemNum_one, int elemNum_two)
{
    if (elemNum_one == elemNum_two)
        return;
    std::vector<int> elemNum_ElemOne;      // 创建一个容器，用来储存第一个网格单元的相邻网格单元编号
    std::vector<int> elemNum_ElemOne_neig; // 创建一个容器，用来储存第一个网格单元在其相邻网格单元的neig中位置
    std::vector<int> elemNum_ElemTwo;      // 创建一个容器，用来储存第二个网格单元的相邻网格单元编号
    std::vector<int> elemNum_ElemTwo_neig; // 创建一个容器，用来储存第二个网格单元在其相邻网格单元的neig中位置
    _MESH_PROCESS mesh_process;
    for (int i = 0; i < DIM + 1; i++)
        if (su_mesh->elem.at(elemNum_one).neig[i] != -1)
        {
            elemNum_ElemOne.push_back(su_mesh->elem.at(elemNum_one).neig[i]);
            elemNum_ElemOne_neig.push_back(mesh_process.AdjacentElem_pos(su_mesh->elem.at(elemNum_ElemOne.back()), elemNum_one));
        }
    for (int i = 0; i < DIM + 1; i++)
        if (su_mesh->elem.at(elemNum_two).neig[i] != -1)
        {
            elemNum_ElemTwo.push_back(su_mesh->elem.at(elemNum_two).neig[i]);
            elemNum_ElemTwo_neig.push_back(mesh_process.AdjacentElem_pos(su_mesh->elem.at(elemNum_ElemTwo.back()), elemNum_two));
        }
    // 修改elem容器内值
    for (std::vector<int>::iterator iter = elemNum_ElemOne.begin(); iter != elemNum_ElemOne.end(); ++iter)
        su_mesh->elem.at(*iter).neig[elemNum_ElemOne_neig.at(0 + std::distance(elemNum_ElemOne.begin(), iter))] = elemNum_two;
    for (std::vector<int>::iterator iter = elemNum_ElemTwo.begin(); iter != elemNum_ElemTwo.end(); ++iter)
        su_mesh->elem.at(*iter).neig[elemNum_ElemTwo_neig.at(0 + std::distance(elemNum_ElemTwo.begin(), iter))] = elemNum_one;
    // 修改node容器内值
    for (int i = 0; i < DIM + 1; i++)
    {
        su_mesh->node.at(su_mesh->elem.at(elemNum_one).form[i]).elem = elemNum_two;
        su_mesh->node.at(su_mesh->elem.at(elemNum_two).form[i]).elem = elemNum_one;
    }
    // 替换单元位置
    std::swap(su_mesh->elem.at(elemNum_one), su_mesh->elem.at(elemNum_two));
    return;
}

void _BOUNDARY_RECOVERY::Removal_LastNode(_SU_MESH *su_mesh)
{
    // 待删除节点永远是node容器内最后一个元素，直接删除就行
    su_mesh->node.pop_back();
    su_mesh->node_num -= 1;
    return;
}

void _BOUNDARY_RECOVERY::Removal_LastElem(_SU_MESH *su_mesh, int elemNum_after)
{
    ELEM elem_tp = su_mesh->elem.back(); // 储存su_mesh->elem容器内最后一个网格单元元素
    int elemNum_neig;                    // 声明一个变量，用来储存当前网格单元的相邻节点编号
    _MESH_PROCESS mesh_process;
    for (int i = 0; i < DIM + 1; i++)
    {
        elemNum_neig = elem_tp.neig[i];
        if (elemNum_neig != -1 && elemNum_neig < su_mesh->elem_num)
        {
            // 先更新待删除单元的所有节点的elem信息
            for (int j = 0; j < DIM + 1; j++)
            {
                if (j == i)
                    continue;
                // 如果该节点的elem值还是无效，即大于 elemNum_after，则继续更改其elem值
                if (su_mesh->node.at(elem_tp.form[j]).elem >= elemNum_after)
                    su_mesh->node.at(elem_tp.form[j]).elem = elemNum_neig;
            }
            su_mesh->elem.at(elemNum_neig).neig[mesh_process.AdjacentElem_pos(su_mesh->elem.at(elemNum_neig), su_mesh->elem_num - 1)] = -1;
        }
    }
    su_mesh->elem.pop_back();
    su_mesh->elem_num -= 1;
    return;
}

void _BOUNDARY_RECOVERY::Removal_NodeElem(_SU_MESH *su_mesh, std::vector<int> nodeNum_Remove, std::vector<int> elemNum_Remove)
{
    int cnt_node = 0; // 记录待删除节点的个数
    // 为简化删除节点流程，当查找到待删除节点时，使用容器内最后一个有效节点来替换此位置，在程序结束时，删除最后cnt_node个节点，最大限度地保证了整个网格的相邻信息不变
    // 为避免不必要的麻烦，将nodeNum_Remove降序排序，从nodeNum_Remove的第一个节点编号开始检索替换
    sort(nodeNum_Remove.begin(), nodeNum_Remove.end(), std::greater<>());
    // 将nodeNum_Remove内所有节点放到node容器最后
    for (int i = 0; i < int(nodeNum_Remove.size()); i++)
    {
        // 将该节点与node容器的最后一个有效节点交换位置
        // 如果该节点就位于node容器的最后一个有效节点位置，则不交换
        if (nodeNum_Remove.at(i) == su_mesh->node_num - 1 - cnt_node)
        {
            cnt_node++;
            continue;
        }
        else
            ReplaceNode_two(su_mesh, nodeNum_Remove.at(i), su_mesh->node_num - 1 - cnt_node++);
    }
    int cnt_elem = 0; // 记录待删除网格单元的个数
    // 将elemNum_Remove内所有网格单元放到elem容器最后
    sort(elemNum_Remove.begin(), elemNum_Remove.end(), std::greater<>());
    for (int i = 0; i < int(elemNum_Remove.size()); i++)
    {
        // 将该网格单元与elem容器的最后一个有效网格单元交换位置
        // 如果该网格单元就位于elem容器的最后一个有效网格单元位置，则不交换
        if (elemNum_Remove.at(i) == su_mesh->elem_num - 1 - cnt_elem)
        {
            cnt_elem++;
            continue;
        }
        ReplaceElem_two(su_mesh, elemNum_Remove.at(i), su_mesh->elem_num - 1 - cnt_elem++);
    }
    // 由于节点是直接删除，不需要更新相邻信息，所以先删除网格单元
    int elemNum_after = su_mesh->elem_num - cnt_elem; // 储存删除elemNum_Remove内所有单元后网格单元数，避免node的elem值错误
    for (int i = 0; i < cnt_elem; i++)
        Removal_LastElem(su_mesh, elemNum_after);
    for (int i = 0; i < cnt_node; i++)
        Removal_LastNode(su_mesh);
    return;
}

void _BOUNDARY_RECOVERY::Removal_ExGrid(_SU_MESH *su_mesh) // 去掉外部单元并缩减容器
{
    std::vector<int> nodeNum_Remove;
    std::vector<int> elemNum_Remove;
    // 声明一个容器，用来储存待移除网格单元编号

    //if (type == 1)
    //{
    //    std::copy(su_mesh->Delaunay_Frame_numPos, su_mesh->Delaunay_Frame_numPos + 8, nodeNum_Remove); // 声明数组，用来指向储存待移除节点的连续内存空间
    //    for (int i = 0; i < su_mesh->elem_num; i++)
    //        if (su_mesh->elem.at(i).form[3] >= su_mesh->InitNode_num)
    //            elemNum_Remove.push_back(i);
    //}
    //if (type == 2)
    External_Elem_Lookup(su_mesh, &nodeNum_Remove, &elemNum_Remove);
    Removal_NodeElem(su_mesh, nodeNum_Remove, elemNum_Remove); // 在node容器内删除节点与单元并更新所有信息
    return;
}

std::vector<Pathl> _BOUNDARY_RECOVERY::FindPath(_SU_MESH *su_mesh, EDGE edge_recovery)
{
    // 取出待恢复边界边的两个节点
    NODE edge_recovery_node1 = su_mesh->node.at(edge_recovery.form[0]), edge_recovery_node2 = su_mesh->node.at(edge_recovery.form[1]);
    std::vector<Pathl> path; // 待恢复边界边的路径
    // 首先查找包含待恢复边界边第一个节点的Ball
    std::vector<int> elemNum_IncludeNode;
    _MESH_PROCESS mesh_process;
    mesh_process.FindBall_fast(su_mesh, edge_recovery.form[0], &elemNum_IncludeNode);
    std::vector<Pathl> path_tp1; // 存储候选路径元
    // 遍历elemNum_IncludeNode，初始化候选路径元
    for (std::vector<int>::iterator iter = elemNum_IncludeNode.begin(); iter != elemNum_IncludeNode.end(); ++iter)
    {
        Pathl pathl_tp;
        pathl_tp.elem_num = *iter;
        pathl_tp = su_mesh->elem.at(pathl_tp.elem_num);
        pathl_tp.type[0] = 1;
        pathl_tp.pot[0] = edge_recovery_node1;
        pathl_tp.node_num[0] = edge_recovery.form[0]; // 点只需要1个节点编号
        path_tp1.push_back(pathl_tp);
    }
    // 查找路径
    int elemNum_tp, nodeNum_tp;
    bool flag = false;
    bool intersection_judge = false;
    ELEM elem_tp;
    FACE face_tp, face_inintersection;
    EDGE edge_tp, edge_inintersection;
    Point intersection_point;
    std::vector<Pathl> path_tp2; // 存储候选路径元
    std::vector<int> elemNum_IncludeEdge;
    _DATA_PROCESS data_process;
    while (!flag)
    {
        std::vector<Pathl>().swap(path_tp2); // 初始化path_tp2，并释放容器空间
        if (path_tp1.empty())
            break;
        intersection_judge = false;
        for (std::vector<Pathl>::iterator iter = path_tp1.begin(); iter != path_tp1.end(); ++iter)
        {
            // 先得到当前路径元所代表的网格单元
            elemNum_tp = iter->elem_num;
            elem_tp = su_mesh->elem.at(elemNum_tp);
            // 判断路径是否查找完毕，若当前路径元包含待恢复边界边的第二个点，代表在这个方向上路径查找到头了，存入当前路径元，并直接开始下一循环
            if (mesh_process.Elem_Include_Node(elem_tp, edge_recovery.form[1]) != -1)
            {
                iter->type[1] = 1;
                iter->pot[1] = edge_recovery_node2;
                iter->node_num[iter->type[0]] = edge_recovery.form[1]; // 点只需要1个节点编号
                path.push_back(*iter);                                 // 当前路径元是有效路径元，存入路径
                flag = true;
                continue;
            }
            // 查找当前路径元与待恢复边界边的第二个相交图形，根据第一个相交图形种类来决定接下来的相交判断，并按照以下规则进行处理
            // 1.若不存在第二个相交图形且第一个相交图形是点则跳过当前路径元
            // 2.若不存在第二个相交图形且第一个相交图形是线则将可以判断完后不管有没有第二个相交图形，都直接将当前路径元压入path
            // 3.边的相交优先级高于面，毕竟与边相交就一定与面相交，这里的面相交其实主要指的是与面内部相交
            // 有必要说明的是，path_tp1内所有路径元都一定与待恢复边界边相交，哪怕只是一个顶点的相交，所以type[0]的值必定有效
            // 并且从路径的特性上来讲，若第一个相交图形是点，则该点必定是待恢复边界边的第一个点，同理，若第二个相交图形是点，则该点必定是待恢复边界边的第二个点
            // 在开始查找第二个相交图形之前，可以先通过判断当前路径元与待恢复边界边是否有两个相交图形，来加速判断过程
            if (data_process.Edge_Elem_Intersection(edge_recovery_node1, edge_recovery_node2,
                                                    su_mesh->node.at(elem_tp.form[0]), su_mesh->node.at(elem_tp.form[1]),
                                                    su_mesh->node.at(elem_tp.form[2]), su_mesh->node.at(elem_tp.form[3])) == 1)
                continue;
            if (iter->type[0] == -1 || iter->type[0] == 0)
            {
                std::cout << "Path lookup failed!\n";
                system("pause");
            }
            // 若第一个相交图形是点，该点必是待恢复边界边第一个节点，需要判断当前路径元内与该点相对的3条网格边与1个网格面跟待恢复边界边的相交情况
            else if (iter->type[0] == 1)
            {
                intersection_judge = false;
                // 先判断相对网格面的3条网格边，由于第一个相交图形是点，所以直接用待恢复边界边的第一个节点编号来查找相对网格面
                face_tp = mesh_process.Node_Opposite_Face(elem_tp, edge_recovery.form[0]);
                for (int i = 0; i < 2; i++)
                {
                    for (int j = i + 1; j < 3; j++)
                    {
                        // 如果当前判断网格边与待恢复边界边相交，则储存其相交信息
                        if (data_process.Edge_Edge_Intersection(&intersection_point, edge_recovery_node1, edge_recovery_node2,
                                                                su_mesh->node.at(face_tp.form[i]), su_mesh->node.at(face_tp.form[j])))
                        {
                            iter->type[1] = 2;
                            iter->pot[1] = intersection_point;
                            // 边需要2个节点编号
                            iter->node_num[iter->type[0] + 0] = face_tp.form[i];
                            iter->node_num[iter->type[0] + 1] = face_tp.form[j];
                            path.push_back(*iter);     // 当前路径元是有效路径元，存入路径
                            intersection_judge = true; // intersection_judge置为真，表示已经找到相交图形
                        }
                        if (intersection_judge)
                            break;
                    }
                    if (intersection_judge)
                        break;
                }
                // 如果遍历完3条网格边后还未找到相交图形，则再判断相对网格面
                if (!intersection_judge)
                    if (data_process.Edge_Face_Intersection(&intersection_point, edge_recovery_node1, edge_recovery_node2,
                                                            su_mesh->node.at(face_tp.form[0]), su_mesh->node.at(face_tp.form[1]), su_mesh->node.at(face_tp.form[2])))
                    {
                        iter->type[1] = 3;
                        iter->pot[1] = intersection_point;
                        for (int i = 0; i < 3; i++)
                            iter->node_num[iter->type[0] + i] = face_tp.form[i]; // 面需要3个节点编号
                        path.push_back(*iter);                                   // 当前路径元是有效路径元，存入路径
                    }
                if (iter->type[1] == -1)
                    iter->type[1] = 0;
            }
            // 若第一个相交图形是边，则需要判断当前路径元内与该边相对的5条网格边与2个网格面跟待恢复边界边的相交情况
            else if (iter->type[0] == 2)
            {
                intersection_judge = false;
                // 先判断相对的5条网格边
                for (int i = 0; i < 3; i++)
                {
                    for (int j = i + 1; j < 4; j++)
                    {
                        // 跳过已经判断的相交边
                        if (elem_tp.form[i] == iter->node_num[0] && elem_tp.form[j] == iter->node_num[1])
                            continue;
                        // 如果当前判断网格边与待恢复边界边相交，则储存其相交信息
                        if (data_process.Edge_Edge_Intersection(&intersection_point, edge_recovery_node1, edge_recovery_node2,
                                                                su_mesh->node.at(elem_tp.form[i]), su_mesh->node.at(elem_tp.form[j])))
                        {
                            iter->type[1] = 2;
                            iter->pot[1] = intersection_point;
                            // 边需要2个节点编号
                            iter->node_num[iter->type[0] + 0] = elem_tp.form[i];
                            iter->node_num[iter->type[0] + 1] = elem_tp.form[j];
                            intersection_judge = true; // intersection_judge置为真，表示已经找到相交图形
                        }
                        if (intersection_judge)
                            break;
                    }
                    if (intersection_judge)
                        break;
                }
                // 如果没有找到相交图形，则继续判断剩下的2个网格面，先储存下当前已知的相交边
                if (!intersection_judge)
                {
                    edge_inintersection.form[0] = iter->node_num[0];
                    edge_inintersection.form[1] = iter->node_num[1];
                    // 再得到该边的相对边
                    edge_tp = mesh_process.Edge_Opposite_Edge(elem_tp, edge_inintersection);
                    // 分别判断该边的两个相对面
                    for (int i = 0; i < 2; i++)
                    {
                        face_tp.form[0] = edge_tp.form[0];
                        face_tp.form[1] = edge_tp.form[1];
                        face_tp.form[2] = edge_inintersection.form[i];
                        face_tp.Sort();
                        if (data_process.Edge_Face_Intersection(&intersection_point, edge_recovery_node1, edge_recovery_node2,
                                                                su_mesh->node.at(face_tp.form[0]), su_mesh->node.at(face_tp.form[1]), su_mesh->node.at(face_tp.form[2])))
                        {
                            iter->type[1] = 3;
                            iter->pot[1] = intersection_point;
                            for (int i = 0; i < 3; i++)
                                iter->node_num[iter->type[0] + i] = face_tp.form[i]; // 面需要3个节点编号
                            intersection_judge = true;                               // intersection_judge置为真，表示已经找到相交图形
                        }
                        if (intersection_judge)
                            break;
                    }
                }
                if (iter->type[1] == -1)
                    iter->type[1] = 0;
                path.push_back(*iter); // 当前路径元是有效路径元，存入路径
            }
            // 若第一个相交图形是面，则需要判断当前路径元内与该面相对的3条网格边与3个网格面跟待恢复边界边的相交情况
            else
            {
                // 储存下当前已知的相交面
                face_inintersection.form[0] = iter->node_num[0];
                face_inintersection.form[1] = iter->node_num[1];
                face_inintersection.form[2] = iter->node_num[2];
                nodeNum_tp = elem_tp.form[mesh_process.Face_Opposite_Node(elem_tp, face_inintersection)];
                // 先判断相对的3条网格边
                for (int i = 0; i < 3; i++)
                {
                    edge_tp.form[0] = face_inintersection.form[i];
                    edge_tp.form[1] = nodeNum_tp;
                    edge_tp.Sort();
                    // 如果当前判断网格边与待恢复边界边相交，则储存其相交信息
                    if (data_process.Edge_Edge_Intersection(&intersection_point, edge_recovery_node1, edge_recovery_node2,
                                                            su_mesh->node.at(edge_tp.form[0]), su_mesh->node.at(edge_tp.form[1])))
                    {
                        iter->type[1] = 2;
                        iter->pot[1] = intersection_point;
                        // 边需要2个节点编号
                        iter->node_num[iter->type[0] + 0] = edge_tp.form[0];
                        iter->node_num[iter->type[0] + 1] = edge_tp.form[1];
                        path.push_back(*iter);     // 当前路径元是有效路径元，存入路径
                        intersection_judge = true; // intersection_judge置为真，表示已经找到相交图形
                    }
                    if (intersection_judge)
                        break;
                }
                // 如果没有找到相交图形，则继续判断剩下的3个网格面
                if (!intersection_judge)
                {
                    for (int i = 0; i < 2; i++)
                    {
                        for (int j = i + 1; j < 3; j++)
                        {
                            face_tp.form[0] = face_inintersection.form[i];
                            face_tp.form[1] = face_inintersection.form[j];
                            face_tp.form[2] = nodeNum_tp;
                            face_tp.Sort();
                            if (data_process.Edge_Face_Intersection(&intersection_point, edge_recovery_node1, edge_recovery_node2,
                                                                    su_mesh->node.at(face_tp.form[0]), su_mesh->node.at(face_tp.form[1]), su_mesh->node.at(face_tp.form[2])))
                            {
                                iter->type[1] = 3;
                                iter->pot[1] = intersection_point;
                                for (int i = 0; i < 3; i++)
                                    iter->node_num[iter->type[0] + i] = face_tp.form[i]; // 面需要3个节点编号
                                path.push_back(*iter);                                   // 当前路径元是有效路径元，存入路径
                                intersection_judge = true;                               // intersection_judge置为真，表示已经找到相交图形
                            }
                            if (intersection_judge)
                                break;
                        }
                        if (intersection_judge)
                            break;
                    }
                }
                if (iter->type[1] == -1)
                    iter->type[1] = 0;
            }
            // 判断当前路径元与待恢复边界边是否存在第二个相交图形，若未判断则抛出错误，若不存在则直接跳过，若存在则根据该图形种类来扩充path_tp2
            if (iter->type[1] == -1)
            {
                std::cout << "Path lookup failed!\n";
                system("pause");
            }
            else if (iter->type[1] == 0)
                continue;
            // 若第二个相交图形是点，则也可以直接跳过
            else if (iter->type[1] == 1)
                continue;
            // 若第二个相交图形是边，则查找包含该边的所有环（ring），将不存在于当前path_tp1、path_tp2和path的路径元压入path_tp2
            else if (iter->type[1] == 2)
            {
                edge_tp.form[0] = iter->node_num[iter->type[0] + 0];
                edge_tp.form[1] = iter->node_num[iter->type[0] + 1];
                std::vector<int>().swap(elemNum_IncludeEdge);
                mesh_process.FindRing(su_mesh, edge_tp, &elemNum_IncludeEdge, "fast");
                // 遍历elemNum_IncludeEdge，初始化候选路径元
                for (std::vector<int>::iterator elemNum_iter = elemNum_IncludeEdge.begin(); elemNum_iter != elemNum_IncludeEdge.end(); ++elemNum_iter)
                {
                    if (std::find(path_tp1.begin(), path_tp1.end(), *elemNum_iter) == path_tp1.end())
                        if (std::find(path_tp2.begin(), path_tp2.end(), *elemNum_iter) == path_tp2.end())
                            if (std::find(path.begin(), path.end(), *elemNum_iter) == path.end())
                            {
                                Pathl pathl_tp;
                                pathl_tp.elem_num = *elemNum_iter;
                                pathl_tp = su_mesh->elem.at(pathl_tp.elem_num);
                                pathl_tp.type[0] = 2;
                                pathl_tp.pot[0] = iter->pot[1];
                                // 边需要2个节点编号
                                pathl_tp.node_num[0] = edge_tp.form[0];
                                pathl_tp.node_num[1] = edge_tp.form[1];
                                path_tp2.push_back(pathl_tp);
                            }
                }
            }
            // 若第二个相交图形是面，则将当前三角化内与当前路径元相邻，且相邻面是该相交面的路径元直接压入path_tp2
            else
            {
                face_inintersection.form[0] = iter->node_num[iter->type[0] + 0];
                face_inintersection.form[1] = iter->node_num[iter->type[0] + 1];
                face_inintersection.form[2] = iter->node_num[iter->type[0] + 2];
                elemNum_tp = elem_tp.neig[mesh_process.Face_Opposite_Node(elem_tp, face_inintersection)];
                Pathl pathl_tp;
                pathl_tp.elem_num = elemNum_tp;
                pathl_tp = su_mesh->elem.at(pathl_tp.elem_num);
                pathl_tp.type[0] = 3;
                pathl_tp.pot[0] = iter->pot[1];
                // 面需要3个节点编号
                pathl_tp.node_num[0] = face_tp.form[0];
                pathl_tp.node_num[1] = face_tp.form[1];
                pathl_tp.node_num[2] = face_tp.form[2];
                path_tp2.push_back(pathl_tp);
            }
        }
        std::vector<Pathl>().swap(path_tp1); // 初始化path_tp1，并释放容器空间
        path_tp1 = path_tp2;
    }
    return path;
}

void _BOUNDARY_RECOVERY::Decompose_Pathl(std::vector<Pathl> *path)
{
    _MESH_PROCESS mesh_process;
    for (std::vector<Pathl>::iterator pathl = path->begin(); pathl != path->end(); ++pathl)
    {
        // 通过路径元type域值，判断当前路径元类型，对不同类型进行不同形式的分解
        // 单边型（包含点边型，边点型）
        if ((pathl->type[0] == 1 && pathl->type[1] == 2) || (pathl->type[0] == 2 && pathl->type[1] == 0) || (pathl->type[0] == 2 && pathl->type[1] == 1))
        {
            // 单边型会分解成两个网格单元
            pathl->Decom_elem_num = 2;
            pathl->Decom_elem = (ELEM *)malloc(sizeof(ELEM) * pathl->Decom_elem_num);
            // 取出待分解的那条网格边，对于单边和点边型、边点型，其获得方式不同
            EDGE ExplodeEdge;
            // 点边型
            if (pathl->type[0] == 1)
                memcpy(ExplodeEdge.form, pathl->node_num + 1, sizeof(ExplodeEdge.form));
            // 单边型，边点型
            else
                memcpy(ExplodeEdge.form, pathl->node_num, sizeof(ExplodeEdge.form));
            // 得到待分解网格边在当前路径元内的相对网格边
            EDGE ExplodeOppoEdge = mesh_process.Edge_Opposite_Edge(*pathl, ExplodeEdge);
            // 得到分解后的两个网格单元
            if (pathl->Decom_elem)
            {
                *(pathl->Decom_elem + 0) = ELEM(ExplodeOppoEdge.form[0], ExplodeOppoEdge.form[1], ExplodeEdge.form[0], STEINER_NOD, -1, -1, -1, pathl->neig[mesh_process.Elem_Include_Node(*pathl, ExplodeEdge.form[1])]);
                *(pathl->Decom_elem + 1) = ELEM(ExplodeOppoEdge.form[0], ExplodeOppoEdge.form[1], ExplodeEdge.form[1], STEINER_NOD, -1, -1, -1, pathl->neig[mesh_process.Elem_Include_Node(*pathl, ExplodeEdge.form[0])]);
            }
        }
        // 对边型和邻边型的两个相交图形都是边
        else if (pathl->type[0] == 2 && pathl->type[1] == 2)
        {
            // 首先得到待分解的两条网格边
            EDGE ExplodeEdge_1, ExplodeEdge_2;
            memcpy(ExplodeEdge_1.form, pathl->node_num, sizeof(ExplodeEdge_1.form));
            memcpy(ExplodeEdge_2.form, pathl->node_num + 2, sizeof(ExplodeEdge_2.form));
            // 通过待分解边的节点编号来分辨对边型和邻边型
            // 邻边型的两条待分解边包含一个相同节点，对边型则没有相同节点
            // 首先判断两条待分解边是否包含相同节点，若包含，则记录该相同节点编号，再记录 在该两条待分解边组成的网格面中 该相同节点相对的网格边
            int ExplodeSameNode_num = -1;
            bool adjacent_judge = false;
            EDGE OppoEdge;
            for (int i = 0; i < 2; i++)
            {
                for (int j = 0; j < 2; j++)
                    if (ExplodeEdge_1.form[i] == ExplodeEdge_2.form[j])
                    {
                        ExplodeSameNode_num = ExplodeEdge_1.form[i];
                        adjacent_judge = true;
                        OppoEdge.form[0] = ExplodeEdge_1.form[!i];
                        OppoEdge.form[1] = ExplodeEdge_2.form[!j];
                    }
            }
            // adjacent_judge为真则代表是邻边型
            if (adjacent_judge)
            {
                // 得到与这两条待分解边组成的网格面在当前路径元内相对的节点编号
                FACE face_tp{ExplodeSameNode_num, OppoEdge.form[0], OppoEdge.form[1]};
                int ExplodeOppoNode_num = pathl->form[mesh_process.Face_Opposite_Node(*pathl, face_tp)];
                // 邻边型路径元分解时要先查找路径内与当前路径元以该网格面相邻的路径元（也是邻边型）的分解方式，若还未分解，则当前路径元以“S”型进行分解，否则以相反类型进行分解
                // 邻边型路径元的分解具有两种类型，“S”型和“Z”型，这两种类型相互拓扑相容
                int elemNum_tp = pathl->neig[mesh_process.Face_Opposite_Node(*pathl, face_tp)];
                std::vector<Pathl>::iterator pathl_tp = std::find(path->begin(), path->end(), elemNum_tp);
                if (pathl_tp == path->end())
                    std::cout << "Decompose_Pathl run error, the guess is that the path lookup failed!\n", system("pause");
                else
                {
                    if (pathl_tp->Decom_type_two_sides == 'S')
                        pathl->Decom_type_two_sides = 'Z';
                    else
                        pathl->Decom_type_two_sides = 'S';
                }
                // 邻边型会分解成三个网格单元
                pathl->Decom_elem_num = 3;
                pathl->Decom_elem = (ELEM *)malloc(sizeof(ELEM) * pathl->Decom_elem_num);
                // 用OppoEdge来判断正方向
                // 这里为了后续网格单元生成的准确性与便利性，应将路径元的节点顺序与OppoEdge的顺序相对应
                if (OppoEdge.form[0] > OppoEdge.form[1])
                {
                    OppoEdge.Sort();
                    std::swap(*(pathl->pot + 0), *(pathl->pot + 1));
                }
                // 得到分解后的三个网格单元，由于前面修改了OppoEdge，这里“S”型与“Z”型分解一致
                if (pathl->Decom_elem)
                {
                    if (pathl->Decom_type_two_sides == 'S' || pathl->Decom_type_two_sides == 'Z')
                    {
                        *pathl->Decom_elem = ELEM(ExplodeSameNode_num, ExplodeOppoNode_num, STEINER_NOD, STEINER_NOD, -1, -1, -1, -1);
                        *(pathl->Decom_elem + 1) = ELEM(OppoEdge.form[1], ExplodeOppoNode_num, STEINER_NOD, STEINER_NOD, -1, -1, -1, -1);
                        *(pathl->Decom_elem + 2) = ELEM(OppoEdge.form[0], OppoEdge.form[1], ExplodeOppoNode_num, STEINER_NOD, -1, -1, -1, pathl->neig[mesh_process.Elem_Include_Node(*pathl, ExplodeSameNode_num)]);
                    }
                    //else if (pathl->Decom_type_two_sides == 'Z')
                    //{
                    //    *pathl->Decom_elem = ELEM(ExplodeSameNode_num, ExplodeOppoNode_num, STEINER_NOD, STEINER_NOD, -1, -1, -1, -1);
                    //    *(pathl->Decom_elem + 1) = ELEM(OppoEdge.form[1], ExplodeOppoNode_num, STEINER_NOD, STEINER_NOD, -1, -1, -1, -1);
                    //    *(pathl->Decom_elem + 2) = ELEM(OppoEdge.form[0], OppoEdge.form[1], ExplodeOppoNode_num, STEINER_NOD, -1, -1, -1, pathl->neig[mesh_process.Elem_Include_Node(*pathl, ExplodeSameNode_num)]);
                    //}
                    else
                        std::cout << "Bilateral pathl's Decom_type_two_sides set error!\n", system("pause");
                }
            }
            // 否则则是对边型
            else
            {
                pathl->Decom_type_two_sides = 'D';
                // 对边型会分解成四个网格单元
                pathl->Decom_elem_num = 4;
                pathl->Decom_elem = (ELEM *)malloc(sizeof(ELEM) * pathl->Decom_elem_num);
                // 得到分解后的四个网格单元
                if (pathl->Decom_elem)
                {
                    *pathl->Decom_elem = ELEM(ExplodeEdge_1.form[0], ExplodeEdge_2.form[0], STEINER_NOD, STEINER_NOD, -1, -1, -1, -1);
                    *(pathl->Decom_elem + 1) = ELEM(ExplodeEdge_1.form[0], ExplodeEdge_2.form[1], STEINER_NOD, STEINER_NOD, -1, -1, -1, -1);
                    *(pathl->Decom_elem + 2) = ELEM(ExplodeEdge_1.form[1], ExplodeEdge_2.form[0], STEINER_NOD, STEINER_NOD, -1, -1, -1, -1);
                    *(pathl->Decom_elem + 3) = ELEM(ExplodeEdge_1.form[1], ExplodeEdge_2.form[1], STEINER_NOD, STEINER_NOD, -1, -1, -1, -1);
                }
            }
        }
        // 点面型（面点型）
        else if ((pathl->type[0] == 1 && pathl->type[1] == 3) || (pathl->type[0] == 3 && pathl->type[1] == 1))
        {
            // 点面型（面点型）会分解成三个网格单元
            pathl->Decom_elem_num = 3;
            pathl->Decom_elem = (ELEM *)malloc(sizeof(ELEM) * pathl->Decom_elem_num);
            // 取出待分解的那个网格面，点面型、面点型，其获得方式不同
            FACE ExplodeFace;
            // 点面型
            if (pathl->type[0] == 1)
                memcpy(ExplodeFace.form, pathl->node_num + 1, sizeof(ExplodeFace.form));
            // 面点型
            else
                memcpy(ExplodeFace.form, pathl->node_num, sizeof(ExplodeFace.form));
            // 得到待分解网格面在当前路径元内的相对网格节点编号
            int ExplodeOppoNode_num = pathl->form[mesh_process.Face_Opposite_Node(*pathl, ExplodeFace)];
            // 得到分解后的三个网格单元
            if (pathl->Decom_elem)
            {
                *pathl->Decom_elem = ELEM(ExplodeOppoNode_num, ExplodeFace.form[0], ExplodeFace.form[1], STEINER_NOD, -1, -1, -1, pathl->neig[mesh_process.Elem_Include_Node(*pathl, ExplodeFace.form[2])]);
                *(pathl->Decom_elem + 1) = ELEM(ExplodeOppoNode_num, ExplodeFace.form[0], ExplodeFace.form[2], STEINER_NOD, -1, -1, -1, pathl->neig[mesh_process.Elem_Include_Node(*pathl, ExplodeFace.form[1])]);
                *(pathl->Decom_elem + 2) = ELEM(ExplodeOppoNode_num, ExplodeFace.form[1], ExplodeFace.form[2], STEINER_NOD, -1, -1, -1, pathl->neig[mesh_process.Elem_Include_Node(*pathl, ExplodeFace.form[0])]);
            }
        }
        // 边面型（面边型）
        else if ((pathl->type[0] == 2 && pathl->type[1] == 3) || (pathl->type[0] == 3 && pathl->type[1] == 2))
        {
            // 边面型（面边型）会分解成四个网格单元
            pathl->Decom_elem_num = 4;
            pathl->Decom_elem = (ELEM *)malloc(sizeof(ELEM) * pathl->Decom_elem_num);
            // 取出待分解的那个网格面，边面型、面边型，其获得方式不同，同时取出待分解的网格边
            EDGE ExplodeEdge;
            FACE ExplodeFace;
            // 边面型
            if (pathl->type[0] == 2)
            {
                memcpy(ExplodeEdge.form, pathl->node_num, sizeof(ExplodeEdge.form));
                memcpy(ExplodeFace.form, pathl->node_num + 2, sizeof(ExplodeFace.form));
            }
            // 面边型
            else
            {
                memcpy(ExplodeFace.form, pathl->node_num, sizeof(ExplodeFace.form));
                memcpy(ExplodeEdge.form, pathl->node_num + 3, sizeof(ExplodeEdge.form));
            }
            // 得到待分解网格面在当前路径元内的相对网格节点编号
            int ExplodeOppoNode_num = pathl->form[mesh_process.Face_Opposite_Node(*pathl, ExplodeFace)];
            // 得到待分解网格面与待分解网格边同时包含的节点编号
            int ExplodeSameNode_num = ExplodeOppoNode_num == ExplodeEdge.form[0] ? ExplodeEdge.form[1] : ExplodeEdge.form[0];
            // 得到待分解网格边在当前路径元内的相对网格边
            EDGE ExplodeOppoEdge = mesh_process.Edge_Opposite_Edge(*pathl, ExplodeEdge);
            // 得到分解后的四个网格单元
            if (pathl->Decom_elem)
            {
                *pathl->Decom_elem = ELEM(ExplodeSameNode_num, ExplodeOppoEdge.form[0], STEINER_NOD, STEINER_NOD, -1, -1, -1, -1);
                *(pathl->Decom_elem + 1) = ELEM(ExplodeSameNode_num, ExplodeOppoEdge.form[1], STEINER_NOD, STEINER_NOD, -1, -1, -1, -1);
                *(pathl->Decom_elem + 2) = ELEM(ExplodeOppoEdge.form[0], ExplodeOppoEdge.form[1], STEINER_NOD, STEINER_NOD, -1, -1, -1, -1);
                *(pathl->Decom_elem + 3) = ELEM(ExplodeOppoNode_num, ExplodeOppoEdge.form[0], ExplodeOppoEdge.form[1], STEINER_NOD, -1, -1, -1, pathl->neig[mesh_process.Elem_Include_Node(*pathl, ExplodeSameNode_num)]);
            }
        }
        // 双面型
        else if (pathl->type[0] == 3 && pathl->type[1] == 3)
        {
            // 双面型会分解成五个网格单元
            pathl->Decom_elem_num = 5;
            pathl->Decom_elem = (ELEM *)malloc(sizeof(ELEM) * pathl->Decom_elem_num);
            // 取出待分解的两个网格面
            FACE ExplodeFace_1, ExplodeFace_2;
            memcpy(ExplodeFace_1.form, pathl->node_num, sizeof(ExplodeFace_1.form));
            memcpy(ExplodeFace_2.form, pathl->node_num + 3, sizeof(ExplodeFace_2.form));
            // 得到两个待分解网格面的相邻网格边
            EDGE ExplodeAdjacentEdge = mesh_process.face_AdjacentEdge(ExplodeFace_1, ExplodeFace_2);
            // 得到在第一个网格面内与该相邻网格边相对的网格节点编号
            int ExplodeFaceOppoNode_num = mesh_process.Face_Opposite_Node(ExplodeFace_1, ExplodeAdjacentEdge);
            // 得到在路径元内与第一个网格面相对的网格节点编号
            int PathlOppoNode_num = pathl->form[mesh_process.Face_Opposite_Node(*pathl, ExplodeFace_1)];
            // 得到分解后的五个网格单元
            if (pathl->Decom_elem)
            {
                *(pathl->Decom_elem + 0) = ELEM(ExplodeAdjacentEdge.form[0], ExplodeAdjacentEdge.form[1], STEINER_NOD, STEINER_NOD, -1, -1, -1, -1);
                *(pathl->Decom_elem + 1) = ELEM(ExplodeFaceOppoNode_num, ExplodeAdjacentEdge.form[0], STEINER_NOD, STEINER_NOD, -1, -1, -1, -1);
                *(pathl->Decom_elem + 2) = ELEM(ExplodeFaceOppoNode_num, ExplodeAdjacentEdge.form[1], STEINER_NOD, STEINER_NOD, -1, -1, -1, -1);
                *(pathl->Decom_elem + 3) = ELEM(ExplodeFaceOppoNode_num, PathlOppoNode_num, ExplodeAdjacentEdge.form[0], STEINER_NOD, -1, -1, -1, pathl->neig[mesh_process.Elem_Include_Node(*pathl, ExplodeAdjacentEdge.form[1])]);
                *(pathl->Decom_elem + 4) = ELEM(ExplodeFaceOppoNode_num, PathlOppoNode_num, ExplodeAdjacentEdge.form[1], STEINER_NOD, -1, -1, -1, pathl->neig[mesh_process.Elem_Include_Node(*pathl, ExplodeAdjacentEdge.form[0])]);
            }
        }
        else
        {
            std::cout << "Pathl decompose error!\n";
            system("pause");
        }
    }
    return;
}

void _BOUNDARY_RECOVERY::Repair_Path(double shortest_dis, std::vector<Pathl> *path)
{
    _MESH_PROCESS mesh_process;
    bool repair_judgment = false; // 定义一个变量，用于判断该路径是否需要修复
    // 目前主要是对双面型路径元进行修复，将其转换为单边型路径元，避免由于双面型路径元两个面上的相交交点距离过近，远远小于模型整体网格量度，导致误差的出现
    int face_face_pathl = 0; // 定义一个变量，用于路径内储存双面型路径元数量
    // 首先查找双面型路径元，再对其两个相交交点间距离进行判断
    for (std::vector<Pathl>::iterator pathl = path->begin(); pathl != path->end(); ++pathl)
    {
        if (pathl->type[0] == 3 && pathl->type[1] == 3)
        {
            // 利用网格模型最短边界边长度来判定是否需要对该路径进行修复
            if (pathl->get_pot_distance() < shortest_dis * 0.5)
                repair_judgment = true;
            face_face_pathl++;
        }
        else
            continue;
    }
    if (repair_judgment && face_face_pathl == int(path->size() - 2))
    {
        EDGE edge_common;    // 查找一个共同边
        Point average_point; // 储存所有相交点的平均节点位置
        edge_common = mesh_process.face_AdjacentEdge(FACE(path->at(0).node_num[1], path->at(0).node_num[2], path->at(0).node_num[3]), FACE(path->at(1).node_num[3], path->at(1).node_num[4], path->at(1).node_num[5]));
        edge_common.Sort();
        for (std::vector<Pathl>::iterator pathl = path->begin() + 1; pathl != path->end(); ++pathl)
        {
            if (mesh_process.Face_Opposite_Node(FACE(pathl->node_num[0], pathl->node_num[1], pathl->node_num[2]), edge_common) == -1)
                std::cout << "Repair path run error, check the accuracy of the path!\n", system("pause");
            average_point = average_point + pathl->pot[0];
        }
        // 得到所有相交点的平均节点位置
        average_point = average_point / double(path->size() - 1);
        // 修复所有双面型路径元
        for (std::vector<Pathl>::iterator pathl = path->begin(); pathl != path->end(); ++pathl)
        {
            if (pathl == path->begin())
            {
                pathl->type[1] = 2;
                pathl->pot[1] = average_point;
                memcpy(pathl->node_num + 1, edge_common.form, sizeof(edge_common.form));
            }
            else if (pathl == path->end() - 1)
            {
                pathl->type[0] = 2;
                pathl->pot[0] = average_point;
                memcpy(pathl->node_num, edge_common.form, sizeof(edge_common.form));
                *(pathl->node_num + 2) = *(pathl->node_num + 3);
                *(pathl->node_num + 3) = -1;
            }
            else
            {
                pathl->type[0] = 2, pathl->type[1] = 0;
                pathl->pot[0] = average_point;
                for (int i = 0; i < 6; i++)
                    pathl->node_num[i] = -1;
                memcpy(pathl->node_num, edge_common.form, sizeof(edge_common.form));
            }
        }
    }
    return;
}

void _BOUNDARY_RECOVERY::Pathl_Generate_GridCell(_SU_MESH *su_mesh, std::vector<Pathl> *path, EDGE edge_recovery)
{
    _MESH_PROCESS mesh_process;
    //_DATA_PROCESS data_process;
    // 声明两个迭代器
    int elemNum_iter = -1;
    std::vector<FACE>::iterator face_iter;
    FACE *face_judge = nullptr;
    int *elemNum_judge = nullptr;
    int faceNum_cnt = 0;
    Pathl pathl;
    // 声明两个容器，分别存储待更新相邻信息的网格面与相对应的网格单元编号
    std::vector<int> elemNum_adjacent;
    std::vector<FACE> face_adjacent;
    // 声明一个数组，储存路径上节点编号
    int *path_nodeNum = (int *)malloc(sizeof(int) * (path->size() + 2));
    int path_nodeNum_iter = 0;
    *(path_nodeNum + path_nodeNum_iter++) = edge_recovery.form[0];
    for (std::vector<Pathl>::iterator path_iter = path->begin(); path_iter != path->end(); ++path_iter)
    {
        pathl = *path_iter;
        // 通过路径元type域值，判断当前路径元类型，对不同类型进行不同形式的网格生成过程
        // 以下代码中若涉及到直接修改相邻信息的语句，其网格单元的顺序确定都是按照Decompose_Pathl()函数中路径元分解时确定的顺序来的
        // 首先会根据以上所述顺序直接更新路径元内部网格单元之间的相邻信息和路径元与普通网格单元之间的相邻信息，再更新路径元与路径元之间的相邻信息
        // 路径元与路径元之间相邻信息的更新会频繁利用到网格面的查找
        // 单边型（包含点边型，边点型）
        if ((pathl.type[0] == 1 && pathl.type[1] == 2) || (pathl.type[0] == 2 && pathl.type[1] == 0) || (pathl.type[0] == 2 && pathl.type[1] == 1))
        {
            // 单边型、点边型、边点型的网格生成步骤相同，但其steiner点的储存位置不同
            // 首先将steiner点压入node容器，并储存该steiner点在网格中的节点编号
            // 先判断当前待压入的steiner点是否已被压入
            int steiner_node_num = -1;
            if (pathl.type[0] == 1)
            {
                if (su_mesh->node.back() == NODE(*(pathl.pot + 1)))
                    steiner_node_num = su_mesh->node_num - 1;
                else
                {
                    su_mesh->node.push_back(NODE(*(pathl.pot + 1)));
                    su_mesh->node.back().spac = mesh_process.get_aver_spac(su_mesh, su_mesh->elem.at(pathl.elem_num));
                    steiner_node_num = su_mesh->node_num++;
                    // 根据steiner点编号，更新路径节点信息
                    *(path_nodeNum + path_nodeNum_iter++) = steiner_node_num;
                }
            }
            else
            {
                if (su_mesh->node.back() == NODE(*pathl.pot))
                    steiner_node_num = su_mesh->node_num - 1;
                else
                {
                    su_mesh->node.push_back(NODE(*pathl.pot));
                    su_mesh->node.back().spac = mesh_process.get_aver_spac(su_mesh, su_mesh->elem.at(pathl.elem_num));
                    steiner_node_num = su_mesh->node_num++;
                    // 根据steiner点编号，更新路径节点信息
                    *(path_nodeNum + path_nodeNum_iter++) = steiner_node_num;
                }
            }
            // 根据steiner点编号，更新路径元分解生成的网格单元的节点信息
            (pathl.Decom_elem + 0)->form[3] = steiner_node_num;
            (pathl.Decom_elem + 1)->form[3] = steiner_node_num;
            su_mesh->node.at(steiner_node_num).elem = pathl.elem_num;
            // 先更新所有能在该步骤下进行更新的相邻信息，再将新生成的两个网格单元插入elem容器
            // 得到新生成的网格单元使用的网格单元编号
            int pathl_elem_num[] = {pathl.elem_num, su_mesh->elem_num};
            // 更新这两个网格单元的相邻信息
            (pathl.Decom_elem + 0)->neig[2] = pathl_elem_num[1];
            //su_mesh->elem.at(pathl.Decom_elem->neig[3]).neig[mesh_process.AdjacentElem_pos(su_mesh->elem.at(pathl.Decom_elem->neig[3]), pathl.elem_num)] = pathl.elem_num;
            (pathl.Decom_elem + 1)->neig[2] = pathl_elem_num[0];
            su_mesh->elem.at((pathl.Decom_elem + 1)->neig[3]).neig[mesh_process.AdjacentElem_pos(su_mesh->elem.at((pathl.Decom_elem + 1)->neig[3]), pathl.elem_num)] = pathl_elem_num[1];
            // 单边型、点边型、边点型会有四个待判断的网格面
            faceNum_cnt = 4;
            face_judge = new FACE[4]{{(pathl.Decom_elem + 0)->form[0], (pathl.Decom_elem + 0)->form[2], (pathl.Decom_elem + 0)->form[3]},
                                     {(pathl.Decom_elem + 0)->form[1], (pathl.Decom_elem + 0)->form[2], (pathl.Decom_elem + 0)->form[3]},
                                     {(pathl.Decom_elem + 1)->form[0], (pathl.Decom_elem + 1)->form[2], (pathl.Decom_elem + 1)->form[3]},
                                     {(pathl.Decom_elem + 1)->form[1], (pathl.Decom_elem + 1)->form[2], (pathl.Decom_elem + 1)->form[3]}};
            // 这四个网格面相对应的网格单元编号如下
            elemNum_judge = new int[4]{pathl_elem_num[0], pathl_elem_num[0], pathl_elem_num[1], pathl_elem_num[1]};
            // 将新生成的两个网格单元在elem容器内替换掉路径元所代表网格单元所在位置,或者压入elem
            (pathl.Decom_elem + 0)->Sort();
            (pathl.Decom_elem + 1)->Sort();
            su_mesh->elem.at(pathl.elem_num) = *(pathl.Decom_elem + 0);
            su_mesh->elem.push_back(*(pathl.Decom_elem + 1));
            su_mesh->elem_num++;
            for (int i = 0; i < pathl.Decom_elem_num; i++)
                mesh_process.Renew_NodeElem(su_mesh, pathl_elem_num[i]);
        }
        // 对边型和邻边型的两个相交图形都是边
        else if (pathl.type[0] == 2 && pathl.type[1] == 2)
        {
            // 首先将steiner点压入node容器，并储存该steiner点在网格中的节点编号
            // 先判断当前待压入的steiner点是否已被压入
            int *steiner_node_num = new int[2]{-1, -1};
            for (int i = 0; i < 2; i++)
            {
                if (*(su_mesh->node.end() - 1) == NODE(*(pathl.pot + i)))
                    *(steiner_node_num + i) = su_mesh->node_num - 1;
                else if (*(su_mesh->node.end() - 2) == NODE(*(pathl.pot + i)))
                    *(steiner_node_num + i) = su_mesh->node_num - 2;
                else
                {
                    su_mesh->node.push_back(NODE(*(pathl.pot + i)));
                    su_mesh->node.back().spac = mesh_process.get_aver_spac(su_mesh, su_mesh->elem.at(pathl.elem_num));
                    *(steiner_node_num + i) = su_mesh->node_num++;
                    // 根据steiner点编号，更新路径节点信息
                    *(path_nodeNum + path_nodeNum_iter++) = *(steiner_node_num + i);
                }
            }
            // 根据steiner点编号，更新路径元分解生成的网格单元的节点信息，然后更新相邻信息，插入elem容器，最后储存待判断网格面
            // 对边型，“D”型
            if (pathl.Decom_type_two_sides == 'D')
            {
                // 首先更新节点信息
                for (int i = 0; i < 4; i++)
                    (pathl.Decom_elem + i)->form[2] = *(steiner_node_num + 0), (pathl.Decom_elem + i)->form[3] = *(steiner_node_num + 1);
                su_mesh->node.at(*(steiner_node_num + 0)).elem = pathl.elem_num, su_mesh->node.at(*(steiner_node_num + 1)).elem = pathl.elem_num;
                // 得到新生成的网格单元使用的网格单元编号
                int pathl_elem_num[] = {pathl.elem_num, su_mesh->elem_num, su_mesh->elem_num + 1, su_mesh->elem_num + 2};
                // 更新相邻信息
                (pathl.Decom_elem + 0)->neig[0] = pathl_elem_num[2], (pathl.Decom_elem + 0)->neig[1] = pathl_elem_num[1];
                (pathl.Decom_elem + 1)->neig[0] = pathl_elem_num[3], (pathl.Decom_elem + 1)->neig[1] = pathl_elem_num[0];
                (pathl.Decom_elem + 2)->neig[0] = pathl_elem_num[0], (pathl.Decom_elem + 2)->neig[1] = pathl_elem_num[3];
                (pathl.Decom_elem + 3)->neig[0] = pathl_elem_num[1], (pathl.Decom_elem + 3)->neig[1] = pathl_elem_num[2];
                // 对边型会有八个待判断的网格面
                faceNum_cnt = 8;
                face_judge = new FACE[8];   // 储存八个网格面
                elemNum_judge = new int[8]; // 储存这八个网格面相对应的网格单元编号
                for (int i = 0; i < 4; i++)
                {
                    *(face_judge + 2 * i) = FACE{(pathl.Decom_elem + i)->form[0], (pathl.Decom_elem + i)->form[1], (pathl.Decom_elem + i)->form[2]};
                    *(face_judge + 2 * i + 1) = FACE{(pathl.Decom_elem + i)->form[0], (pathl.Decom_elem + i)->form[1], (pathl.Decom_elem + i)->form[3]};
                    *(elemNum_judge + 2 * i) = pathl_elem_num[i];
                    *(elemNum_judge + 2 * i + 1) = pathl_elem_num[i];
                    (pathl.Decom_elem + i)->Sort();
                }
                // 将新生成的四个网格单元在elem容器内替换掉路径元所代表网格单元所在位置,或者压入elem
                su_mesh->elem.at(pathl.elem_num) = *(pathl.Decom_elem + 0);
                for (int i = 1; i < 4; i++)
                {
                    su_mesh->elem.push_back(*(pathl.Decom_elem + i));
                    su_mesh->elem_num++;
                }
                for (int i = 0; i < pathl.Decom_elem_num; i++)
                    mesh_process.Renew_NodeElem(su_mesh, pathl_elem_num[i]);
            }
            // 邻边“S”型与“Z”型一致
            else if (pathl.Decom_type_two_sides == 'S' || pathl.Decom_type_two_sides == 'Z')
            {
                // 首先更新节点信息
                for (int i = 0; i < 2; i++)
                    (pathl.Decom_elem + i)->form[2] = *(steiner_node_num + 0), (pathl.Decom_elem + i)->form[3] = *(steiner_node_num + 1);
                (pathl.Decom_elem + 2)->form[3] = *(steiner_node_num + 0);
                su_mesh->node.at(*(steiner_node_num + 0)).elem = pathl.elem_num, su_mesh->node.at(*(steiner_node_num + 1)).elem = pathl.elem_num;
                // 得到新生成的网格单元使用的网格单元编号
                int pathl_elem_num[] = {pathl.elem_num, su_mesh->elem_num, su_mesh->elem_num + 1};
                // 更新相邻信息
                (pathl.Decom_elem + 0)->neig[0] = pathl_elem_num[1];
                (pathl.Decom_elem + 1)->neig[0] = pathl_elem_num[0], (pathl.Decom_elem + 1)->neig[3] = pathl_elem_num[2];
                (pathl.Decom_elem + 2)->neig[0] = pathl_elem_num[1];
                su_mesh->elem.at((pathl.Decom_elem + 2)->neig[3]).neig[mesh_process.AdjacentElem_pos(su_mesh->elem.at((pathl.Decom_elem + 2)->neig[3]), pathl.elem_num)] = pathl_elem_num[2];
                // 邻边型会有七个待判断的网格面
                faceNum_cnt = 7;
                face_judge = new FACE[7];   // 储存七个网格面
                elemNum_judge = new int[7]; // 储存这七个网格面相对应的网格单元编号
                *(face_judge + 0) = FACE{(pathl.Decom_elem + 0)->form[0], (pathl.Decom_elem + 0)->form[1], (pathl.Decom_elem + 0)->form[2]};
                *(elemNum_judge + 0) = pathl_elem_num[0];
                for (int i = 0; i < 3; i++)
                {
                    *(face_judge + 2 * i + 1) = FACE{(pathl.Decom_elem + i)->form[0], (pathl.Decom_elem + i)->form[1], (pathl.Decom_elem + i)->form[3]};
                    *(face_judge + 2 * i + 2) = FACE{(pathl.Decom_elem + i)->form[0], (pathl.Decom_elem + i)->form[2], (pathl.Decom_elem + i)->form[3]};
                    *(elemNum_judge + 2 * i + 1) = pathl_elem_num[i];
                    *(elemNum_judge + 2 * i + 2) = pathl_elem_num[i];
                    (pathl.Decom_elem + i)->Sort();
                }
                // 将新生成的三个网格单元在elem容器内替换掉路径元所代表网格单元所在位置,或者压入elem
                su_mesh->elem.at(pathl.elem_num) = *(pathl.Decom_elem + 0);
                for (int i = 1; i < 3; i++)
                {
                    su_mesh->elem.push_back(*(pathl.Decom_elem + i));
                    su_mesh->elem_num++;
                }
                for (int i = 0; i < pathl.Decom_elem_num; i++)
                    mesh_process.Renew_NodeElem(su_mesh, pathl_elem_num[i]);
            }
            //// 邻边“Z”型
            //else if (pathl.Decom_type_two_sides == 'Z')
            //{
            //    // 首先更新节点信息
            //    for (int i = 0; i < 2; i++)
            //        (pathl.Decom_elem + i)->form[2] = *(steiner_node_num + 0), (pathl.Decom_elem + i)->form[3] = *(steiner_node_num + 1);
            //    (pathl.Decom_elem + 2)->form[3] = *(steiner_node_num + 0);
            //    su_mesh->node.at(*(steiner_node_num + 0)).elem = pathl.elem_num, su_mesh->node.at(*(steiner_node_num + 1)).elem = pathl.elem_num;
            //    // 得到新生成的网格单元使用的网格单元编号
            //    int pathl_elem_num[] = {pathl.elem_num, su_mesh->elem_num, su_mesh->elem_num + 1};
            //    // 更新相邻信息
            //    (pathl.Decom_elem + 0)->neig[0] = pathl_elem_num[1];
            //    (pathl.Decom_elem + 1)->neig[0] = pathl_elem_num[0], (pathl.Decom_elem + 1)->neig[3] = pathl_elem_num[2];
            //    (pathl.Decom_elem + 2)->neig[0] = pathl_elem_num[1];
            //    su_mesh->elem.at((pathl.Decom_elem + 2)->neig[3]).neig[mesh_process.AdjacentElem_pos(su_mesh->elem.at((pathl.Decom_elem + 2)->neig[3]), pathl.elem_num)] = pathl_elem_num[2];
            //    // 邻边型会有七个待判断的网格面
            //    faceNum_cnt = 7;
            //    face_judge = new FACE[7];   // 储存七个网格面
            //    elemNum_judge = new int[7]; // 储存这七个网格面相对应的网格单元编号
            //    *(face_judge + 0) = FACE{(pathl.Decom_elem + 0)->form[0], (pathl.Decom_elem + 0)->form[1], (pathl.Decom_elem + 0)->form[2]};
            //    *(elemNum_judge + 0) = pathl_elem_num[0];
            //    for (int i = 0; i < 3; i++)
            //    {
            //        *(face_judge + 2 * i + 1) = FACE{(pathl.Decom_elem + i)->form[0], (pathl.Decom_elem + i)->form[1], (pathl.Decom_elem + i)->form[3]};
            //        *(face_judge + 2 * i + 2) = FACE{(pathl.Decom_elem + i)->form[0], (pathl.Decom_elem + i)->form[2], (pathl.Decom_elem + i)->form[3]};
            //        *(elemNum_judge + 2 * i + 1) = pathl_elem_num[i];
            //        *(elemNum_judge + 2 * i + 2) = pathl_elem_num[i];
            //        (pathl.Decom_elem + i)->Sort();
            //    }
            //    // 将新生成的三个网格单元在elem容器内替换掉路径元所代表网格单元所在位置,或者压入elem
            //    su_mesh->elem.at(pathl.elem_num) = *(pathl.Decom_elem + 0);
            //    for (int i = 1; i < 3; i++)
            //    {
            //        su_mesh->elem.push_back(*(pathl.Decom_elem + i));
            //        su_mesh->elem_num++;
            //    }
            //    for (int i = 0; i < pathl.Decom_elem_num; i++)
            //        mesh_process.Renew_NodeElem(su_mesh, pathl_elem_num[i]);
            //}
            else
                std::cout << "Bilateral pathl's Decom_type_two_sides set error!\n", system("pause");
            delete[] steiner_node_num;
        }
        // 点面型（面点型）
        else if ((pathl.type[0] == 1 && pathl.type[1] == 3) || (pathl.type[0] == 3 && pathl.type[1] == 1))
        {
            // 点面型、面点型的网格生成步骤相同，但其steiner点的储存位置不同
            // 首先将steiner点压入node容器，并储存该steiner点在网格中的节点编号
            // 先判断当前待压入的steiner点是否已被压入
            int steiner_node_num = -1;
            if (pathl.type[0] == 1)
            {
                if (su_mesh->node.back() == NODE(*(pathl.pot + 1)))
                    steiner_node_num = su_mesh->node_num - 1;
                else
                {
                    su_mesh->node.push_back(NODE(*(pathl.pot + 1)));
                    su_mesh->node.back().spac = mesh_process.get_aver_spac(su_mesh, su_mesh->elem.at(pathl.elem_num));
                    steiner_node_num = su_mesh->node_num++;
                    // 根据steiner点编号，更新路径节点信息
                    *(path_nodeNum + path_nodeNum_iter++) = steiner_node_num;
                }
            }
            else
            {
                if (su_mesh->node.back() == NODE(*pathl.pot))
                    steiner_node_num = su_mesh->node_num - 1;
                else
                {
                    su_mesh->node.push_back(NODE(*pathl.pot));
                    su_mesh->node.back().spac = mesh_process.get_aver_spac(su_mesh, su_mesh->elem.at(pathl.elem_num));
                    steiner_node_num = su_mesh->node_num++;
                    // 根据steiner点编号，更新路径节点信息
                    *(path_nodeNum + path_nodeNum_iter++) = steiner_node_num;
                }
            }
            // 根据steiner点编号，更新路径元分解生成的网格单元的节点信息
            for (int i = 0; i < 3; i++)
                (pathl.Decom_elem + i)->form[3] = steiner_node_num;
            su_mesh->node.at(steiner_node_num).elem = pathl.elem_num;
            // 先更新所有能在该步骤下进行更新的相邻信息，再将新生成的两个网格单元插入elem容器
            // 得到新生成的网格单元使用的网格单元编号
            int pathl_elem_num[] = {pathl.elem_num, su_mesh->elem_num, su_mesh->elem_num + 1};
            // 更新这三个网格单元的相邻信息
            (pathl.Decom_elem + 0)->neig[1] = pathl_elem_num[2], (pathl.Decom_elem + 0)->neig[2] = pathl_elem_num[1];
            (pathl.Decom_elem + 1)->neig[1] = pathl_elem_num[2], (pathl.Decom_elem + 1)->neig[2] = pathl_elem_num[0];
            su_mesh->elem.at((pathl.Decom_elem + 1)->neig[3]).neig[mesh_process.AdjacentElem_pos(su_mesh->elem.at((pathl.Decom_elem + 1)->neig[3]), pathl.elem_num)] = pathl_elem_num[1];
            (pathl.Decom_elem + 2)->neig[1] = pathl_elem_num[1], (pathl.Decom_elem + 2)->neig[2] = pathl_elem_num[0];
            su_mesh->elem.at((pathl.Decom_elem + 2)->neig[3]).neig[mesh_process.AdjacentElem_pos(su_mesh->elem.at((pathl.Decom_elem + 2)->neig[3]), pathl.elem_num)] = pathl_elem_num[2];
            // 点面型、面点型会有三个待判断的网格面
            faceNum_cnt = 3;
            face_judge = new FACE[3]{{(pathl.Decom_elem + 0)->form[1], (pathl.Decom_elem + 0)->form[2], (pathl.Decom_elem + 0)->form[3]},
                                     {(pathl.Decom_elem + 1)->form[1], (pathl.Decom_elem + 1)->form[2], (pathl.Decom_elem + 1)->form[3]},
                                     {(pathl.Decom_elem + 2)->form[1], (pathl.Decom_elem + 2)->form[2], (pathl.Decom_elem + 2)->form[3]}};
            // 这三个网格面相对应的网格单元编号如下
            elemNum_judge = new int[3]{pathl_elem_num[0], pathl_elem_num[1], pathl_elem_num[2]};
            // 将新生成的两个网格单元在elem容器内替换掉路径元所代表网格单元所在位置,或者压入elem
            (pathl.Decom_elem + 0)->Sort();
            (pathl.Decom_elem + 1)->Sort();
            (pathl.Decom_elem + 2)->Sort();
            su_mesh->elem.at(pathl.elem_num) = *(pathl.Decom_elem + 0);
            su_mesh->elem.push_back(*(pathl.Decom_elem + 1));
            su_mesh->elem_num++;
            su_mesh->elem.push_back(*(pathl.Decom_elem + 2));
            su_mesh->elem_num++;
            for (int i = 0; i < pathl.Decom_elem_num; i++)
                mesh_process.Renew_NodeElem(su_mesh, pathl_elem_num[i]);
        }
        // 边面型（面边型）
        else if ((pathl.type[0] == 2 && pathl.type[1] == 3) || (pathl.type[0] == 3 && pathl.type[1] == 2))
        {
            // 边面型、面边型的网格生成步骤相同，但其边steiner点的储存位置不同
            // 首先将steiner点压入node容器，并储存该steiner点在网格中的节点编号
            // 先判断当前待压入的steiner点是否已被压入
            int *steiner_node_num = new int[2]{-1, -1};
            for (int i = 0; i < 2; i++)
            {
                if (*(su_mesh->node.end() - 1) == NODE(*(pathl.pot + i)))
                    *(steiner_node_num + i) = su_mesh->node_num - 1;
                else if (*(su_mesh->node.end() - 2) == NODE(*(pathl.pot + i)))
                    *(steiner_node_num + i) = su_mesh->node_num - 2;
                else
                {
                    su_mesh->node.push_back(NODE(*(pathl.pot + i)));
                    su_mesh->node.back().spac = mesh_process.get_aver_spac(su_mesh, su_mesh->elem.at(pathl.elem_num));
                    *(steiner_node_num + i) = su_mesh->node_num++;
                    // 根据steiner点编号，更新路径节点信息
                    *(path_nodeNum + path_nodeNum_iter++) = *(steiner_node_num + i);
                }
            }
            // 根据steiner点编号，更新路径元分解生成的网格单元的节点信息
            for (int i = 0; i < 3; i++)
                (pathl.Decom_elem + i)->form[2] = *(steiner_node_num + 0), (pathl.Decom_elem + i)->form[3] = *(steiner_node_num + 1);
            if (pathl.type[0] == 2)
                (pathl.Decom_elem + 3)->form[3] = *(steiner_node_num + 0);
            else
                (pathl.Decom_elem + 3)->form[3] = *(steiner_node_num + 1);
            su_mesh->node.at(*(steiner_node_num + 0)).elem = pathl.elem_num, su_mesh->node.at(*(steiner_node_num + 1)).elem = pathl.elem_num;
            // 先更新所有能在该步骤下进行更新的相邻信息，再将新生成的四个网格单元插入elem容器
            // 得到新生成的网格单元使用的网格单元编号
            int pathl_elem_num[] = {pathl.elem_num, su_mesh->elem_num, su_mesh->elem_num + 1, su_mesh->elem_num + 2};
            // 更新这四个网格单元的相邻信息
            (pathl.Decom_elem + 0)->neig[0] = pathl_elem_num[1], (pathl.Decom_elem + 0)->neig[1] = pathl_elem_num[2];
            (pathl.Decom_elem + 1)->neig[0] = pathl_elem_num[1], (pathl.Decom_elem + 1)->neig[1] = pathl_elem_num[0];
            (pathl.Decom_elem + 2)->neig[0] = pathl_elem_num[2], (pathl.Decom_elem + 2)->neig[1] = pathl_elem_num[0];
            if (pathl.type[0] == 2)
                (pathl.Decom_elem + 2)->neig[3] = pathl_elem_num[3];
            else
                (pathl.Decom_elem + 2)->neig[2] = pathl_elem_num[3];
            (pathl.Decom_elem + 3)->neig[0] = pathl_elem_num[2];
            su_mesh->elem.at((pathl.Decom_elem + 3)->neig[3]).neig[mesh_process.AdjacentElem_pos(su_mesh->elem.at((pathl.Decom_elem + 3)->neig[3]), pathl.elem_num)] = pathl_elem_num[3];
            // 边面型、面边型会有七个待判断的网格面
            faceNum_cnt = 7;
            face_judge = new FACE[7];
            elemNum_judge = new int[7];
            for (int i = 0; i < 1; i++)
            {
                *(face_judge + 2 * i + 0) = FACE{(pathl.Decom_elem + i)->form[0], (pathl.Decom_elem + i)->form[1], (pathl.Decom_elem + i)->form[2]};
                *(face_judge + 2 * i + 1) = FACE{(pathl.Decom_elem + i)->form[0], (pathl.Decom_elem + i)->form[1], (pathl.Decom_elem + i)->form[3]};
                *(elemNum_judge + 2 * i + 0) = pathl_elem_num[i];
                *(elemNum_judge + 2 * i + 1) = pathl_elem_num[i];
                (pathl.Decom_elem + i)->Sort();
            }
            if (pathl.type[0] == 2)
                *(face_judge + 4) = FACE{(pathl.Decom_elem + 2)->form[0], (pathl.Decom_elem + 2)->form[1], (pathl.Decom_elem + 2)->form[3]};
            else
                *(face_judge + 4) = FACE{(pathl.Decom_elem + 2)->form[0], (pathl.Decom_elem + 2)->form[1], (pathl.Decom_elem + 2)->form[2]};
            *(elemNum_judge + 4) = pathl_elem_num[2];
            (pathl.Decom_elem + 2)->Sort();
            *(face_judge + 5) = FACE{(pathl.Decom_elem + 3)->form[0], (pathl.Decom_elem + 3)->form[1], (pathl.Decom_elem + 3)->form[3]};
            *(elemNum_judge + 5) = pathl_elem_num[3];
            *(face_judge + 6) = FACE{(pathl.Decom_elem + 3)->form[0], (pathl.Decom_elem + 3)->form[2], (pathl.Decom_elem + 3)->form[3]};
            *(elemNum_judge + 6) = pathl_elem_num[3];
            (pathl.Decom_elem + 3)->Sort();
            // 将新生成的两个网格单元在elem容器内替换掉路径元所代表网格单元所在位置,或者压入elem
            su_mesh->elem.at(pathl.elem_num) = *(pathl.Decom_elem + 0);
            for (int i = 1; i < 3; i++)
            {
                su_mesh->elem.push_back(*(pathl.Decom_elem + i));
                su_mesh->elem_num++;
            }
            for (int i = 0; i < pathl.Decom_elem_num; i++)
                mesh_process.Renew_NodeElem(su_mesh, pathl_elem_num[i]);
            delete[] steiner_node_num;
        }
        // 双面型
        else if (pathl.type[0] == 3 && pathl.type[1] == 3)
        {
            // 首先将steiner点压入node容器，并储存该steiner点在网格中的节点编号
            // 先判断当前待压入的steiner点是否已被压入
            int *steiner_node_num = new int[2]{-1, -1};
            for (int i = 0; i < 2; i++)
            {
                if (*(su_mesh->node.end() - 1) == NODE(*(pathl.pot + i)))
                    *(steiner_node_num + i) = su_mesh->node_num - 1;
                else if (*(su_mesh->node.end() - 2) == NODE(*(pathl.pot + i)))
                    *(steiner_node_num + i) = su_mesh->node_num - 2;
                else
                {
                    su_mesh->node.push_back(NODE(*(pathl.pot + i)));
                    su_mesh->node.back().spac = mesh_process.get_aver_spac(su_mesh, su_mesh->elem.at(pathl.elem_num));
                    *(steiner_node_num + i) = su_mesh->node_num++;
                    // 根据steiner点编号，更新路径节点信息
                    *(path_nodeNum + path_nodeNum_iter++) = *(steiner_node_num + i);
                }
            }
            // 根据steiner点编号，更新路径元分解生成的网格单元的节点信息
            for (int i = 0; i < 3; i++)
                (pathl.Decom_elem + i)->form[2] = *(steiner_node_num + 0), (pathl.Decom_elem + i)->form[3] = *(steiner_node_num + 1);
            su_mesh->node.at(*(steiner_node_num + 0)).elem = pathl.elem_num, su_mesh->node.at(*(steiner_node_num + 1)).elem = pathl.elem_num;
            for (int i = 3; i < 5; i++)
                (pathl.Decom_elem + i)->form[3] = *(steiner_node_num + 1);
            // 先更新所有能在该步骤下进行更新的相邻信息，再将新生成的五个网格单元插入elem容器
            // 得到新生成的网格单元使用的网格单元编号
            int pathl_elem_num[] = {pathl.elem_num, su_mesh->elem_num, su_mesh->elem_num + 1, su_mesh->elem_num + 2, su_mesh->elem_num + 3};
            // 更新这五个网格单元的相邻信息
            (pathl.Decom_elem + 0)->neig[0] = pathl_elem_num[2], (pathl.Decom_elem + 0)->neig[1] = pathl_elem_num[1];
            (pathl.Decom_elem + 1)->neig[0] = pathl_elem_num[0], (pathl.Decom_elem + 1)->neig[1] = pathl_elem_num[2], (pathl.Decom_elem + 1)->neig[2] = pathl_elem_num[3];
            (pathl.Decom_elem + 2)->neig[0] = pathl_elem_num[0], (pathl.Decom_elem + 2)->neig[1] = pathl_elem_num[1], (pathl.Decom_elem + 2)->neig[2] = pathl_elem_num[4];
            (pathl.Decom_elem + 3)->neig[1] = pathl_elem_num[1], (pathl.Decom_elem + 3)->neig[2] = pathl_elem_num[4];
            su_mesh->elem.at((pathl.Decom_elem + 3)->neig[3]).neig[mesh_process.AdjacentElem_pos(su_mesh->elem.at((pathl.Decom_elem + 3)->neig[3]), pathl.elem_num)] = pathl_elem_num[3];
            (pathl.Decom_elem + 4)->neig[1] = pathl_elem_num[2], (pathl.Decom_elem + 4)->neig[2] = pathl_elem_num[3];
            su_mesh->elem.at((pathl.Decom_elem + 4)->neig[3]).neig[mesh_process.AdjacentElem_pos(su_mesh->elem.at((pathl.Decom_elem + 4)->neig[3]), pathl.elem_num)] = pathl_elem_num[4];
            // 单边型、点边型、边点型会有六个待判断的网格面
            faceNum_cnt = 6;
            face_judge = new FACE[6];
            elemNum_judge = new int[6];
            for (int i = 0; i < 3; i++)
            {
                if (i == 0)
                {
                    *(face_judge + i) = FACE{(pathl.Decom_elem + i)->form[0], (pathl.Decom_elem + i)->form[1], (pathl.Decom_elem + i)->form[3]};
                    *(elemNum_judge + i) = pathl_elem_num[i];
                }
                *(face_judge + i + 1) = FACE{(pathl.Decom_elem + i)->form[0], (pathl.Decom_elem + i)->form[1], (pathl.Decom_elem + i)->form[2]};
                *(elemNum_judge + i + 1) = pathl_elem_num[i];
                (pathl.Decom_elem + i)->Sort();
            }
            *(face_judge + 4) = FACE{(pathl.Decom_elem + 3)->form[1], (pathl.Decom_elem + 3)->form[2], (pathl.Decom_elem + 3)->form[3]};
            *(elemNum_judge + 4) = pathl_elem_num[3];
            (pathl.Decom_elem + 3)->Sort();
            *(face_judge + 5) = FACE{(pathl.Decom_elem + 4)->form[1], (pathl.Decom_elem + 4)->form[2], (pathl.Decom_elem + 4)->form[3]};
            *(elemNum_judge + 5) = pathl_elem_num[4];
            (pathl.Decom_elem + 4)->Sort();
            // 将新生成的五个网格单元在elem容器内替换掉路径元所代表网格单元所在位置,或者压入elem
            su_mesh->elem.at(pathl.elem_num) = *(pathl.Decom_elem + 0);
            for (int i = 1; i < 5; i++)
            {
                su_mesh->elem.push_back(*(pathl.Decom_elem + i));
                su_mesh->elem_num++;
            }
            for (int i = 0; i < pathl.Decom_elem_num; i++)
                mesh_process.Renew_NodeElem(su_mesh, pathl_elem_num[i]);
            delete[] steiner_node_num;
        }
        else
            std::cout << "Pathl generate grid cell error!\n", system("pause");
        // 更新路径元与路径元之间的相邻信息
        // 在face_adjacent容器内查找这些网格面，若不存在则直接压入容器并将这个网格面代表的网格单元编号一并压入容器elemNum_adjacent，若存在则取出相对于的网格面与网格单元编号，并进行相邻信息更新
        FACE face_tp;
        int elemNum_tp = -1;
        // 保证数据有效
        if (face_judge == nullptr || elemNum_judge == nullptr)
            std::cout << "Pathl generate grid cell error!\n", system("pause");
        else
            for (int i = 0; i < faceNum_cnt; i++)
            {
                face_judge[i].Sort();
                if ((face_iter = std::find(face_adjacent.begin(), face_adjacent.end(), face_judge[i])) == face_adjacent.end())
                {
                    face_adjacent.push_back(face_judge[i]);
                    elemNum_adjacent.push_back(elemNum_judge[i]);
                }
                else
                {
                    elemNum_iter = std::distance(face_adjacent.begin(), face_iter);
                    if (elemNum_iter < int(elemNum_adjacent.size()))
                        elemNum_tp = elemNum_adjacent.at(elemNum_iter);
                    else
                        std::cout << "Pathl generate grid cell error, the elemNum_iter is out of bounds!\n", system("pause");
                    // 从容器内删除这两个值
                    face_adjacent.erase(face_iter);
                    elemNum_adjacent.erase(elemNum_adjacent.begin() + elemNum_iter);
                    // 修改包含该网格面的两个网格单元的相邻信息
                    su_mesh->elem.at(elemNum_judge[i]).neig[mesh_process.Face_Opposite_Node(su_mesh->elem.at(elemNum_judge[i]), face_judge[i])] = elemNum_tp;
                    su_mesh->elem.at(elemNum_tp).neig[mesh_process.Face_Opposite_Node(su_mesh->elem.at(elemNum_tp), face_judge[i])] = elemNum_judge[i];
                }
            }
        elemNum_iter = -1;
        delete[] face_judge;
        face_judge = nullptr;
        delete[] elemNum_judge;
        elemNum_judge = nullptr;
        faceNum_cnt = 0;
    }
    // 判断face_adjacent是否为空，若不为空，代表路径元分解错误
    if (!face_adjacent.empty())
        std::cout << "When Pathl_Generate_GridCell, the pathl decompose error, some adjacent face were not found successfully!\n", system("pause");
    pathl.Decom_elem = nullptr; // 避免析构函数错误删除内存空间
    // 更新路径节点信息，最后一个节点
    *(path_nodeNum + path_nodeNum_iter++) = edge_recovery.form[1];
    // 储存边界边分段信息
    for (int i = 0; i < path_nodeNum_iter - 1; i++)
        su_mesh->boundary_edge_Decom.push_back(EDGE(*(path_nodeNum + i), *(path_nodeNum + i + 1)));

    //NODE node_tp;
    //int nodeNum_tp;
    //int elemNum_neig_tp[2] = {-1, -1};
    //EDGE edge_tp;
    //ELEM elem_tp;
    //bool merge_judge = false; // 判断上次循环是否进行了合并操作
    //std::vector<int> elemNum_IncludeNode;
    //std::vector<int> elemNum_IncludeEdge;
    //std::vector<int> elemNum_wait_delete;
    //int *nodeNum_wait_delete = new int[path_nodeNum_iter]; // 储存合并结束后需要删除的节点编号
    //int nodeNum_wait_delete_cnt = 0;
    //// 利用路径节点信息，判断这些新插入的steiner点间距离，若过短，则进行合并操作
    //for (int i = 1; i < path_nodeNum_iter - 2; i++)
    //{
    //    // 取出待判断边
    //    if (merge_judge)
    //        edge_tp = EDGE(nodeNum_tp, *(path_nodeNum + i + 1));
    //    else
    //        edge_tp = EDGE(*(path_nodeNum + i), *(path_nodeNum + i + 1));
    //    merge_judge = false;
    //    std::vector<int>().swap(elemNum_IncludeEdge);
    //    mesh_process.FindRing(su_mesh, edge_tp, &elemNum_IncludeEdge, "fast");
    //    if (elemNum_IncludeEdge.empty())
    //        std::cout << "Pathl generate gridCell merge fail, elemNum_IncludeEdge search error!\n", system("pause");
    //    // 通过该边长度判断是否需要合并，低于模型最短边界边的一定倍数值时需要合并
    //    if (data_process.get_dist(su_mesh->node.at(edge_tp.form[0]).pos, su_mesh->node.at(edge_tp.form[1]).pos) <= su_mesh->shortest_border_edge * Max_steiner_point_internal)
    //    {
    //        node_tp = (su_mesh->node.at(edge_tp.form[0]) + su_mesh->node.at(edge_tp.form[1])) * 0.5;
    //        nodeNum_tp = edge_tp.form[0];
    //        *(nodeNum_wait_delete + nodeNum_wait_delete_cnt++) = edge_tp.form[1]; // 待删除节点编号，节点留在整个流程结束后再删除
    //        // elemNum_IncludeEdge内的网格单元在节点合并结束后都会成为无效单元，需要进行删除操作
    //        // 先将这些网格单元替换到elem容器末尾，以便后续删除操作，并记录下这些网格单元在替换后的网格单元编号
    //        std::vector<int>().swap(elemNum_wait_delete);
    //        int cnt_elem = 0; // 记录待删除网格单元的个数
    //        std::sort(elemNum_IncludeEdge.begin(), elemNum_IncludeEdge.end());
    //        for (int j = int(elemNum_IncludeEdge.size()) - 1; j >= 0; j--)
    //        {
    //            // 将该网格单元与elem容器的最后一个有效网格单元交换位置
    //            // 如果该网格单元就位于elem容器的最后一个有效网格单元位置，则不交换
    //            if (elemNum_IncludeEdge.at(j) == su_mesh->elem.size() - 1 - cnt_elem)
    //            {
    //                cnt_elem++;
    //                elemNum_wait_delete.push_back(elemNum_IncludeEdge.at(j));
    //                continue;
    //            }
    //            ReplaceElem_two(su_mesh, elemNum_IncludeEdge.at(j), su_mesh->elem_num - 1 - cnt_elem);
    //            elemNum_wait_delete.push_back(su_mesh->elem_num - 1 - cnt_elem);
    //            cnt_elem++;
    //        }
    //        // 修改合并后的节点信息
    //        su_mesh->node.at(nodeNum_tp) = node_tp;
    //        // 储存待修改节点信息的网格单元
    //        std::vector<int>().swap(elemNum_IncludeNode);
    //        mesh_process.FindBall_fast(su_mesh, edge_tp.form[1], &elemNum_IncludeNode);
    //        // 修改节点合并后相关网格单元的相邻信息
    //        for (std::vector<int>::iterator iter = elemNum_wait_delete.begin(); iter != elemNum_wait_delete.end(); ++iter)
    //        {
    //            elem_tp = su_mesh->elem.at(*iter);
    //            elemNum_neig_tp[0] = elem_tp.neig[mesh_process.ELEM_Include_Node(elem_tp, edge_tp.form[1])];
    //            elemNum_neig_tp[1] = elem_tp.neig[mesh_process.ELEM_Include_Node(elem_tp, edge_tp.form[0])];
    //            su_mesh->elem.at(elemNum_neig_tp[0]).neig[mesh_process.AdjacentElem_pos(su_mesh->elem.at(elemNum_neig_tp[0]), *iter)] = elemNum_neig_tp[1];
    //            su_mesh->elem.at(elemNum_neig_tp[1]).neig[mesh_process.AdjacentElem_pos(su_mesh->elem.at(elemNum_neig_tp[1]), *iter)] = elemNum_neig_tp[0];
    //            mesh_process.Renew_NodeElem(su_mesh, elemNum_neig_tp[0]);
    //            mesh_process.Renew_NodeElem(su_mesh, elemNum_neig_tp[1]);
    //        }
    //        // 删除相关网格单元
    //        for (int k = 0; k < cnt_elem; k++)
    //            su_mesh->elem.pop_back(), su_mesh->elem_num--;
    //        // 修改网格单元节点编号信息
    //        for (std::vector<int>::iterator iter = elemNum_IncludeNode.begin(); iter != elemNum_IncludeNode.end(); ++iter)
    //            if (*iter < su_mesh->elem_num)
    //                su_mesh->elem.at(*iter).form[mesh_process.ELEM_Include_Node(su_mesh->elem.at(*iter), edge_tp.form[1])] = nodeNum_tp, su_mesh->elem.at(*iter).Sort();
    //        merge_judge = true;
    //    }
    //    // 不需要合并的steiner点视为成功插入的steiner点，储存其分段信息
    //    else
    //        su_mesh->boundary_edge_Decom.push_back(edge_tp);
    //}
    //// 删除无效节点，先将其替换到node容器末尾，再进行删除操作
    //std::sort(nodeNum_wait_delete, nodeNum_wait_delete + nodeNum_wait_delete_cnt, std::greater<>());
    //for (int i = 0; i < nodeNum_wait_delete_cnt; i++)
    //{
    //    nodeNum_tp = su_mesh->node_num - 1;
    //    // 如果本来就在末尾，则直接删除
    //    if (*(nodeNum_wait_delete + i) == nodeNum_tp)
    //    {
    //        su_mesh->node.pop_back();
    //        su_mesh->node_num--;
    //        continue;
    //    }
    //    std::vector<int>().swap(elemNum_IncludeNode);
    //    mesh_process.FindBall_fast(su_mesh, nodeNum_tp, &elemNum_IncludeNode); // 查找包含node容器末尾节点的所有网格单元编号
    //    // 修改elem容器内值
    //    for (std::vector<int>::iterator iter = elemNum_IncludeNode.begin(); iter != elemNum_IncludeNode.end(); ++iter)
    //        su_mesh->elem.at(*iter).form[mesh_process.Elem_Include_Node(su_mesh->elem.at(*iter), nodeNum_tp)] = *(nodeNum_wait_delete + i), su_mesh->elem.at(*iter).Sort();
    //    // 替换节点位置
    //    std::swap(su_mesh->node.at(nodeNum_tp), su_mesh->node.at(*(nodeNum_wait_delete + i)));
    //    // 避免初始边界点被修改
    //    if (nodeNum_tp < su_mesh->InitNode_num + 8 || *(nodeNum_wait_delete + i) < su_mesh->InitNode_num + 8)
    //        std::cout << "InitNode being change!\n";
    //    // 在boundary_edge_Decom边界边分段容器内修改节点编号
    //    for (std::vector<EDGE>::iterator iter = su_mesh->boundary_edge_Decom.begin(); iter != su_mesh->boundary_edge_Decom.end(); ++iter)
    //    {
    //        for (int i = 0; i < 2; i++)
    //            if (iter->form[i] == nodeNum_tp)
    //                iter->form[i] = *(nodeNum_wait_delete + i);
    //        iter->Sort();
    //    }
    //    su_mesh->node.pop_back();
    //    su_mesh->node_num--;
    //}
    //delete[] nodeNum_wait_delete;
    //nodeNum_wait_delete = nullptr;
    free(path_nodeNum);
    path_nodeNum = nullptr;
    return;
}

void _BOUNDARY_RECOVERY::Recovery_Boundary_edge(_SU_MESH *su_mesh, EDGE edge_recovery)
{
    // 首先查找待恢复边界边的路径（path），记录其与路径元（pathl）的相交图形与相交点
    std::vector<Pathl> path = FindPath(su_mesh, edge_recovery);
    _MESH_PROCESS mesh_process;
    // 如果路径中只有两个路径元，则可以实现约束边界恢复
    if (path.size() == 2)
    {
        // 再根据第一个路径元的第二个相交图形种类来确定实现当前边界恢复所采用的方式
        // 若该图形是边，则说明这两个路径元全在边界上，属于边界网格单元
        // 则直接通过交换边界边修改网格信息，实现当前边界边恢复
        if (path.front().type[1] == 2)
        {
            // 储存与待恢复边界边相交的那条边
            EDGE edge_tp(path.at(0).node_num[1], path.at(0).node_num[2]);
            // 为加速边界边恢复后网格相邻信息的更新，首先储存边界边恢复前交换域的边界面以及交换域外包含边界面的网格单元编号
            // 并且只需要储存交换域内网格单元在edge_tp两个节点的相对网格面
            std::vector<int> elemNum_adjacent;
            int elemNum_iter;
            std::vector<FACE> face_adjacent;
            std::vector<FACE>::iterator face_iter;
            for (int i = 0; i < 2; i++)
                for (int j = 0; j < 2; j++)
                {
                    elemNum_adjacent.push_back(su_mesh->elem.at(path.at(i).elem_num).neig[mesh_process.ELEM_Include_Node(su_mesh->elem.at(path.at(i).elem_num), edge_tp.form[j])]);
                    face_adjacent.push_back(mesh_process.Node_Opposite_Face(su_mesh->elem.at(path.at(i).elem_num), edge_tp.form[j]));
                }
            // 直接根据节点信息生成两个新的网格，先搜索并储存下最后一个未知的节点编号
            int nodeNum_last = -1;
            for (int i = 0; i < DIM + 1; i++)
                if (su_mesh->elem.at(path.at(0).elem_num).neig[i] == -1)
                {
                    nodeNum_last = su_mesh->elem.at(path.at(0).elem_num).form[i];
                    break;
                }
            ELEM elem_new[2] = {{nodeNum_last, edge_recovery.form[0], edge_recovery.form[1], edge_tp.form[0], -1, -1, -1, path.at(1).elem_num},
                                {nodeNum_last, edge_recovery.form[0], edge_recovery.form[1], edge_tp.form[1], -1, -1, -1, path.at(0).elem_num}};
            FACE face_tp[] = {{nodeNum_last, edge_recovery.form[0], edge_tp.form[0]},
                              {nodeNum_last, edge_recovery.form[1], edge_tp.form[0]},
                              {nodeNum_last, edge_recovery.form[0], edge_tp.form[1]},
                              {nodeNum_last, edge_recovery.form[1], edge_tp.form[1]}};
            for (int i = 0; i < 4; i++)
                face_tp[i].Sort();
            elem_new[0].neig[1] = elemNum_adjacent.at(elemNum_iter = std::distance(face_adjacent.begin(), face_iter = std::find(face_adjacent.begin(), face_adjacent.end(), face_tp[1])));
            elem_new[0].neig[2] = elemNum_adjacent.at(elemNum_iter = std::distance(face_adjacent.begin(), face_iter = std::find(face_adjacent.begin(), face_adjacent.end(), face_tp[0])));
            elem_new[1].neig[1] = elemNum_adjacent.at(elemNum_iter = std::distance(face_adjacent.begin(), face_iter = std::find(face_adjacent.begin(), face_adjacent.end(), face_tp[3])));
            elem_new[1].neig[2] = elemNum_adjacent.at(elemNum_iter = std::distance(face_adjacent.begin(), face_iter = std::find(face_adjacent.begin(), face_adjacent.end(), face_tp[2])));
            elem_new[0].Sort();
            elem_new[1].Sort();
            // 用elem_new替换掉elem容器内值
            su_mesh->elem.at(path.at(0).elem_num) = elem_new[0];
            su_mesh->elem.at(path.at(1).elem_num) = elem_new[1];
            int value_tp;
            // 修改所有相邻信息
            for (int i = 0; i < 2; i++)
            {
                for (int j = 0; j < DIM + 1; j++)
                {
                    if (elem_new[i].neig[j] == -1)
                        continue;
                    value_tp = mesh_process.Face_Opposite_Node(su_mesh->elem.at(elem_new[i].neig[j]), mesh_process.Node_Opposite_Face(elem_new[i], elem_new[i].form[j]));
                    if (value_tp == -1)
                    {
                        std::cout << "Recovery_Boundary_edge run error, please check the program!\n";
                        system("pause");
                    }
                    su_mesh->elem.at(elem_new[i].neig[j]).neig[value_tp] = path.at(i).elem_num;
                }
                // 修改单元所有节点的elem值
                mesh_process.Renew_NodeElem(su_mesh, path.at(i).elem_num);
            }
        }
        // 若该图形是面，则可以直接进行T23变换，实现当前边界恢复
        else if (path.front().type[1] == 3)
        {
            // 储存与待恢复边界边相交的那个面
            FACE face_tp(path.at(0).node_num[1], path.at(0).node_num[2], path.at(0).node_num[3]);
            _QUALITY quality;
            quality.Face_Transform_23(su_mesh, face_tp);
        }
        // 若都不是，则报错
        else
        {
            std::cout << "Recovery_Boundary_edge run error!\n";
            system("pause");
        }
    }
    else
    {
        // 遍历每个路径元，分别进行分解和生成网格单元，同时确保相邻信息的准确性
        // 先对路径进行修复操作
        //Repair_Path(su_mesh->shortest_border_edge, &path);
        // 再对路径元进行分解操作
        Decompose_Pathl(&path);
        // 依据路径中各个路径元类型，将分解后的网格压入elem容器，形成路径元的完整分解生成过程
        Pathl_Generate_GridCell(su_mesh, &path, edge_recovery);
        //mesh_process.Judge_the_validity_of_information(su_mesh);
    }
    return;
}

std::vector<Setl> _BOUNDARY_RECOVERY::FindSet(_SU_MESH *su_mesh, FACE face_recovery)
{
    _MESH_PROCESS mesh_process;
    _DATA_PROCESS data_process;
    // 取出待恢复边界面的三个节点
    NODE face_recovery_node[] = {su_mesh->node.at(face_recovery.form[0]), su_mesh->node.at(face_recovery.form[1]), su_mesh->node.at(face_recovery.form[2])};
    std::vector<Setl> set;          // 待恢复边界面的集
    std::vector<Setl> set_tp1;      // 存储候选集元
    std::vector<int> elemNum_judge; // 储存判断过的网格单元编号
    // 查找包含待恢复边界面每个节点的Ball，并将其中的网格单元编号
    std::vector<int> elemNum_IncludeNode;
    Setl setl_tp;
    for (int i = 0; i < 3; i++)
    {
        std::vector<int>().swap(elemNum_IncludeNode);
        mesh_process.FindBall_fast(su_mesh, face_recovery.form[i], &elemNum_IncludeNode);
        // 遍历elemNum_IncludeNode，初始化候选集元
        for (std::vector<int>::iterator iter = elemNum_IncludeNode.begin(); iter != elemNum_IncludeNode.end(); ++iter)
            if (std::find(elemNum_judge.begin(), elemNum_judge.end(), *iter) == elemNum_judge.end())
            {
                setl_tp.elem_num = *iter;
                setl_tp = su_mesh->elem.at(setl_tp.elem_num);
                set_tp1.push_back(setl_tp);
                elemNum_judge.push_back(*iter);
            }
    }
    // 查找集
    int elemNum_tp;
    ELEM elem_tp;
    bool vertex_node_judge;    // 顶点判断
    int vertex_node_symbol[4]; // 与网格单元的form数组编号一一对应，判断对应位置的顶点是否已被储存
    EDGE edge_tp;
    Point intersection_point;
    int intersec_cnt, vertex_cnt; // 两个计数器
    NODE node_tp[2];
    std::vector<Setl> set_tp2; // 存储候选集元
    std::vector<int> elemNum_IncludeEdge;
    while (!set_tp1.empty())
    {
        std::vector<Setl>().swap(set_tp2); // 初始化set_tp2，并释放容器空间
        for (std::vector<Setl>::iterator iter = set_tp1.begin(); iter != set_tp1.end(); ++iter)
        {
            // 先得到当前候选集元所代表的网格单元
            elemNum_tp = iter->elem_num;
            elem_tp = su_mesh->elem.at(elemNum_tp);
            memset(vertex_node_symbol, 0, sizeof(vertex_node_symbol));
            intersec_cnt = 0, vertex_cnt = 0;
            // 为避免过多误差，导致相交判断出错，这里先用三角形面积判断集元的顶点是否在三角形内部（待恢复边界面就是三角形），若是，则直接储存顶点
            // 并且判断集元的顶点是否是待恢复边界面的顶点，若是，则直接储存顶点
            // 上述判断完成后，在当前网格单元的四个顶点中排除掉上述已被储存的顶点，若剩下顶点数大于等于2，则再将 这些剩下顶点组成的网格单元边 与 待恢复边界面 进行相交判断
            for (int i = 0; i < 4; i++)
            {
                vertex_node_judge = false;
                // 先判断当前顶点是否已被储存
                for (int j = 0; j < vertex_cnt; j++)
                    if (iter->vertex_nodeNum[j] == elem_tp.form[i])
                        vertex_node_judge = true;
                if (vertex_node_judge)
                    continue;
                // 判断当前顶点是否是待恢复边界面的顶点
                if (elem_tp.form[i] == face_recovery.form[0] || elem_tp.form[i] == face_recovery.form[1] || elem_tp.form[i] == face_recovery.form[2])
                    vertex_node_judge = true;
                // 再判断当前顶点是否在待恢复边界面内部
                if (data_process.point_internal_triangle(face_recovery_node[0], face_recovery_node[1], face_recovery_node[2], su_mesh->node.at(elem_tp.form[i])))
                    vertex_node_judge = true;
                if (vertex_node_judge)
                {
                    vertex_node_symbol[i] = 1;
                    iter->intersec_num++;
                    iter->vertex_num++;
                    iter->vertex_nodeNum[vertex_cnt] = elem_tp.form[i];
                    iter->contact_edge[vertex_cnt * 2] = elem_tp.form[i], iter->contact_edge[vertex_cnt * 2 + 1] = (i == 0 ? elem_tp.form[1] : elem_tp.form[0]);
                    vertex_cnt++;
                }
                if (vertex_cnt == 3)
                    break;
            }
            // 查找集元剩下顶点组成的网格边与待恢复边界面的相交情况
            for (int i = 0; i < 4; i++)
                for (int j = i + 1; j < 4; j++)
                {
                    // 跳过已被储存的顶点
                    if (vertex_node_symbol[i] == 1 || vertex_node_symbol[j] == 1)
                        continue;
                    // 取出待判断边
                    edge_tp = EDGE(elem_tp.form[i], elem_tp.form[j]);
                    node_tp[0] = su_mesh->node.at(edge_tp.form[0]), node_tp[1] = su_mesh->node.at(edge_tp.form[1]);
                    vertex_node_judge = false;
                    // 进行待判断边与待恢复边界点相交情况判断
                    if (data_process.Edge_Face_Intersection(&intersection_point, node_tp[0], node_tp[1], face_recovery_node[0], face_recovery_node[1], face_recovery_node[2]))
                    {
                        //for (int m = 0; m < 2; m++)
                        //    if (intersection_point == node_tp[m])
                        //    {
                        //        if (iter->vertex_nodeNum[0] == edge_tp.form[m] || iter->vertex_nodeNum[1] == edge_tp.form[m] || iter->vertex_num == 3)
                        //        {
                        //            vertex_node_judge = true;
                        //            continue;
                        //        }
                        //        iter->intersec_num++;
                        //        iter->vertex_num++;
                        //        iter->vertex_nodeNum[vertex_cnt] = edge_tp.form[m];
                        //        iter->contact_edge[vertex_cnt * 2] = edge_tp.form[0], iter->contact_edge[vertex_cnt * 2 + 1] = edge_tp.form[1];
                        //        vertex_cnt++;
                        //        vertex_node_judge = true;
                        //    }
                        //if (vertex_node_judge)
                        //    continue;
                        iter->intersec_num++;
                        iter->pot[intersec_cnt] = intersection_point;
                        iter->intersec_edge[intersec_cnt * 2] = edge_tp.form[0], iter->intersec_edge[intersec_cnt * 2 + 1] = edge_tp.form[1];
                        intersec_cnt++;
                    }
                }
            // 再一次验证集元的准确性，若是集元，则其intersec_cnt与vertex_cnt相加为3或者4
            if (intersec_cnt + vertex_cnt == 3 || intersec_cnt + vertex_cnt == 4)
            {
                set.push_back(*iter);
                // 对当前集元的每个相交边、接触边，在当前三角化内查找其Ring，并将未判断过的网格单元压入set_tp2
                for (int i = 0; i < intersec_cnt; i++)
                {
                    // 取出待判断边
                    edge_tp = EDGE(iter->intersec_edge[i * 2], iter->intersec_edge[i * 2 + 1]);
                    std::vector<int>().swap(elemNum_IncludeEdge);
                    mesh_process.FindRing(su_mesh, edge_tp, &elemNum_IncludeEdge, "fast");
                    for (std::vector<int>::iterator elemNum_iter = elemNum_IncludeEdge.begin(); elemNum_iter != elemNum_IncludeEdge.end(); ++elemNum_iter)
                        if (std::find(elemNum_judge.begin(), elemNum_judge.end(), *elemNum_iter) == elemNum_judge.end())
                        {
                            setl_tp.elem_num = *elemNum_iter;
                            setl_tp = su_mesh->elem.at(setl_tp.elem_num);
                            set_tp2.push_back(setl_tp);
                            elemNum_judge.push_back(*elemNum_iter);
                        }
                }
                for (int i = 0; i < vertex_cnt; i++)
                {
                    // 取出待判断边
                    edge_tp = EDGE(iter->contact_edge[i * 2], iter->contact_edge[i * 2 + 1]);
                    std::vector<int>().swap(elemNum_IncludeEdge);
                    mesh_process.FindRing(su_mesh, edge_tp, &elemNum_IncludeEdge, "fast");
                    for (std::vector<int>::iterator elemNum_iter = elemNum_IncludeEdge.begin(); elemNum_iter != elemNum_IncludeEdge.end(); ++elemNum_iter)
                        if (std::find(elemNum_judge.begin(), elemNum_judge.end(), *elemNum_iter) == elemNum_judge.end())
                        {
                            setl_tp.elem_num = *elemNum_iter;
                            setl_tp = su_mesh->elem.at(setl_tp.elem_num);
                            set_tp2.push_back(setl_tp);
                            elemNum_judge.push_back(*elemNum_iter);
                        }
                }
            }
        }
        std::vector<Setl>().swap(set_tp1); // 初始化set_tp1，并释放容器空间
        set_tp1 = set_tp2;
    }
    return set;
}

void _BOUNDARY_RECOVERY::Decompose_Setl(std::vector<Setl> *set)
{
    // 集元包含5种类型，分别是0、1、2、3、4条边刺穿待恢复边界面
    _MESH_PROCESS mesh_process;
    for (std::vector<Setl>::iterator setl = set->begin(); setl != set->end(); ++setl)
    {
        // 通过集元setl.intersec_num域与setl.vertex_num域的差值，判断当前集元类型，对不同类型进行不同形式的分解
        // 当没有边刺穿待恢复边界面时，该集元不需要分解
        if (setl->intersec_num - setl->vertex_num == 0)
            continue;
        // 当有1条边刺穿待恢复边界面时，该集元分解方式与边界边恢复中的单边型一致
        else if (setl->intersec_num - setl->vertex_num == 1)
        {
            // 当有1条边刺穿待恢复边界面时，集元会分解成两个网格单元
            setl->Decom_elem_num = 2;
            setl->Decom_elem = (ELEM *)malloc(sizeof(ELEM) * setl->Decom_elem_num);
            // 取出待分解的那条网格边
            EDGE ExplodeEdge{setl->intersec_edge[0], setl->intersec_edge[1]};
            // 得到待分解网格边在当前路径元内的相对网格边
            EDGE ExplodeOppoEdge = mesh_process.Edge_Opposite_Edge(*setl, ExplodeEdge);
            // 得到分解后的两个网格单元
            if (setl->Decom_elem)
            {
                *(setl->Decom_elem + 0) = ELEM(ExplodeOppoEdge.form[0], ExplodeOppoEdge.form[1], ExplodeEdge.form[0], STEINER_NOD, -1, -1, -1, setl->neig[mesh_process.Elem_Include_Node(*setl, ExplodeEdge.form[1])]);
                *(setl->Decom_elem + 1) = ELEM(ExplodeOppoEdge.form[0], ExplodeOppoEdge.form[1], ExplodeEdge.form[1], STEINER_NOD, -1, -1, -1, setl->neig[mesh_process.Elem_Include_Node(*setl, ExplodeEdge.form[0])]);
            }
        }
        // 当有2条边刺穿待恢复边界面时，该集元分解方式与边界边恢复中的邻边型一致
        else if (setl->intersec_num - setl->vertex_num == 2)
        {
            // 首先得到待分解的两条网格边
            EDGE ExplodeEdge_1, ExplodeEdge_2;
            memcpy(ExplodeEdge_1.form, setl->intersec_edge, sizeof(ExplodeEdge_1.form));
            memcpy(ExplodeEdge_2.form, setl->intersec_edge + 2, sizeof(ExplodeEdge_2.form));
            // 邻边型的两条待分解边包含一个相同节点，记录该相同节点编号，再记录 在该两条待分解边组成的网格面中 该相同节点相对的网格边
            int ExplodeSameNode_num = -1;
            EDGE OppoEdge;
            for (int i = 0; i < 2; i++)
            {
                for (int j = 0; j < 2; j++)
                    if (ExplodeEdge_1.form[i] == ExplodeEdge_2.form[j])
                    {
                        ExplodeSameNode_num = ExplodeEdge_1.form[i];
                        OppoEdge.form[0] = ExplodeEdge_1.form[!i];
                        OppoEdge.form[1] = ExplodeEdge_2.form[!j];
                    }
            }
            // 得到与这两条待分解边组成的网格面在当前路径元内相对的节点编号
            FACE face_tp{ExplodeSameNode_num, OppoEdge.form[0], OppoEdge.form[1]};
            int face_pos1 = mesh_process.Face_Opposite_Node(*setl, face_tp), face_pos2 = -1;
            int ExplodeOppoNode_num = setl->form[face_pos1];
            // 邻边型的分解具有两种类型，“S”型和“Z”型，这两种类型相互拓扑相容
            int elemNum_tp = setl->neig[face_pos1];
            std::vector<Setl>::iterator setl_tp = std::find(set->begin(), set->end(), elemNum_tp);
            if (setl_tp == set->end())
                std::cout << "Decompose_Setl run error, the guess is that the set lookup failed!\n", system("pause");
            else
            {
                int face_pos2 = mesh_process.Face_Opposite_Node(*setl_tp, face_tp);
                if (setl_tp->Face_Decom_type[face_pos2] == 'S')
                    setl->Face_Decom_type[face_pos1] = 'Z';
                else
                    setl->Face_Decom_type[face_pos1] = 'S';
            }
            // 邻边型会分解成三个网格单元
            setl->Decom_elem_num = 3;
            setl->Decom_elem = (ELEM *)malloc(sizeof(ELEM) * setl->Decom_elem_num);
            // 用OppoEdge来判断正方向
            // 这里为了后续网格单元生成的准确性与便利性，应将集元的节点顺序与OppoEdge的顺序相对应，正常情况下，OppoEdge是正顺序
            if (OppoEdge.form[0] > OppoEdge.form[1])
            {
                OppoEdge.Sort();
                std::swap(*(setl->pot + 0), *(setl->pot + 1));
                std::cout << "Decompose_Setl, setl->intersec_num - setl->vertex_num == 2, the OppoEdge node order error!\n";
            }
            // 得到分解后的三个网格单元
            if (setl->Decom_elem)
            {
                if (setl->Face_Decom_type[face_pos1] == 'S' || setl->Face_Decom_type[face_pos1] == 'Z')
                {
                    *setl->Decom_elem = ELEM(ExplodeSameNode_num, ExplodeOppoNode_num, STEINER_NOD, STEINER_NOD, -1, -1, -1, -1);
                    *(setl->Decom_elem + 1) = ELEM(OppoEdge.form[1], ExplodeOppoNode_num, STEINER_NOD, STEINER_NOD, -1, -1, -1, -1);
                    *(setl->Decom_elem + 2) = ELEM(OppoEdge.form[0], OppoEdge.form[1], ExplodeOppoNode_num, STEINER_NOD, -1, -1, -1, setl->neig[mesh_process.Elem_Include_Node(*setl, ExplodeSameNode_num)]);
                }
                //else if (setl->Face_Decom_type[face_pos1] == 'Z')
                //{
                //    *setl->Decom_elem = ELEM(ExplodeSameNode_num, ExplodeOppoNode_num, STEINER_NOD, STEINER_NOD, -1, -1, -1, -1);
                //    *(setl->Decom_elem + 1) = ELEM(OppoEdge.form[1], ExplodeOppoNode_num, STEINER_NOD, STEINER_NOD, -1, -1, -1, -1);
                //    *(setl->Decom_elem + 2) = ELEM(OppoEdge.form[0], OppoEdge.form[1], ExplodeOppoNode_num, STEINER_NOD, -1, -1, -1, setl->neig[mesh_process.Elem_Include_Node(*setl, ExplodeSameNode_num)]);
                //}
                else
                    std::cout << "Bilateral pathl's Decom_type_two_sides set error!\n", system("pause");
            }
        }
        // 当有3条边刺穿待恢复边界面时
        else if (setl->intersec_num - setl->vertex_num == 3)
        {
            std::cout << "There are 3 edges piercing the boundary surface to be restored!\n";
            //// 首先得到待分解的三条网格边
            //EDGE ExplodeEdge[3];
            //memcpy(ExplodeEdge[0].form, setl->intersec_edge + 0, sizeof(ExplodeEdge[0].form));
            //memcpy(ExplodeEdge[1].form, setl->intersec_edge + 2, sizeof(ExplodeEdge[1].form));
            //memcpy(ExplodeEdge[2].form, setl->intersec_edge + 4, sizeof(ExplodeEdge[2].form));
            //// 这三条待分解边包含一个相同节点，记录该相同节点编号，再记录在集元内该节点相对的网格边
            //int ExplodeSameNode_num = -1;
            //FACE OppoFace;
            //for (int i = 0; i < 2; i++)
            //    for (int j = 0; j < 2; j++)
            //        if (ExplodeEdge[0].form[i] == ExplodeEdge[1].form[j])
            //            ExplodeSameNode_num = ExplodeEdge[0].form[i];
            //OppoFace = mesh_process.Node_Opposite_Face(*setl, ExplodeSameNode_num);
            //int elemNum_tp = -1;
            //FACE face_tp;
            //int face_pos1 = -1, face_pos2 = -1;
            //std::vector<Setl>::iterator setl_tp;
            //// 这些网格面的分解具有两种类型，“S”型和“Z”型，这两种类型相互拓扑相容
            //// 当有3条边刺穿待恢复边界面时，每个边界面的分解类型遵循一定规律，总的类型分为两种“SSZ”以及"ZZS“
            //// 定义一组映射当编号分别为0、1、2、3、4时，当前网格面的分解类型为”S“、”Z“、”S“、”S“、”Z“
            //// 在进行具体网格面分解前，编号值初始化为2，对三个网格面，每当有网格面按照“S”类型分解，编号值加1；按照“Z”型分解，编号值-1，并且默认分解类型定义为“S”，且按照“SSZ”以及"ZZS“的类型追随分配下去
            //int serial_number = 3;
            //char Decompose_type[] = {'S', 'Z', 'S', 'S', 'Z'};
            //for (int i = 0; i < 3; i++)
            //    for (int j = i + 1; j < 3; j++)
            //    {
            //        face_tp = FACE{OppoFace.form[i], OppoFace.form[j], ExplodeSameNode_num};
            //        face_pos1 = mesh_process.Face_Opposite_Node(*setl, face_tp);
            //        elemNum_tp = setl->neig[face_pos1];
            //        setl_tp = std::find(set->begin(), set->end(), elemNum_tp);
            //        if (setl_tp == set->end())
            //            std::cout << "Decompose_Setl run error, the guess is that the set lookup failed!\n", system("pause");
            //        else
            //        {
            //            int face_pos2 = mesh_process.Face_Opposite_Node(*setl_tp, face_tp);
            //            if (setl_tp->Face_Decom_type[face_pos2] == 'S')
            //                setl->Face_Decom_type[face_pos1] = 'Z', serial_number--;
            //            else if (setl_tp->Face_Decom_type[face_pos2] == 'Z')
            //                setl->Face_Decom_type[face_pos1] = 'S', serial_number++;
            //            else
            //                switch (serial_number)
            //                {
            //                case 0:
            //                    setl->Face_Decom_type[face_pos1] = 'S';
            //                    //serial_number--; // 最后一个
            //                    break;
            //                case 1:
            //                    setl->Face_Decom_type[face_pos1] = 'Z';
            //                    serial_number--;
            //                    break;
            //                case 2:
            //                    setl->Face_Decom_type[face_pos1] = 'S';
            //                    serial_number++;
            //                    break;
            //                case 3:
            //                    setl->Face_Decom_type[face_pos1] = 'S';
            //                    serial_number++;
            //                    break;
            //                case 4:
            //                    setl->Face_Decom_type[face_pos1] = 'Z';
            //                    //serial_number++; // 最后一个
            //                    break;
            //                default:
            //                    std::cout << "Decompose_Setl run error, the serial_number have error value, the vertex_num is 3\n";
            //                    system("pause");
            //                }
            //        }
            //    }
            //std::cout << serial_number << ' ';
            //// 当有3条边刺穿待恢复边界面时，集元会分解成四个网格单元
            //setl->Decom_elem_num = 4;
            //setl->Decom_elem = (ELEM *)malloc(sizeof(ELEM) * setl->Decom_elem_num);
            //// 这里为了后续网格单元生成的准确性与便利性，应将集元的节点顺序与face_tp的顺序相对应
            //// 得到分解后的四个网格单元
            ////if (setl->Decom_elem)
            ////{
            ////    if (setl->Face_Decom_type[face_pos1] == 'S')
            ////    {
            ////        *setl->Decom_elem = ELEM(ExplodeSameNode_num, ExplodeOppoNode_num, STEINER_NOD, STEINER_NOD, -1, -1, -1, -1);
            ////        *(setl->Decom_elem + 1) = ELEM(OppoEdge.form[1], ExplodeOppoNode_num, STEINER_NOD, STEINER_NOD, -1, -1, -1, -1);
            ////        *(setl->Decom_elem + 2) = ELEM(OppoEdge.form[0], OppoEdge.form[1], ExplodeOppoNode_num, STEINER_NOD, -1, -1, -1, setl->neig[mesh_process.Elem_Include_Node(*setl, ExplodeSameNode_num)]);
            ////    }
            ////    else
            ////    {
            ////        *setl->Decom_elem = ELEM(ExplodeSameNode_num, ExplodeOppoNode_num, STEINER_NOD, STEINER_NOD, -1, -1, -1, -1);
            ////        *(setl->Decom_elem + 1) = ELEM(OppoEdge.form[1], ExplodeOppoNode_num, STEINER_NOD, STEINER_NOD, -1, -1, -1, -1);
            ////        *(setl->Decom_elem + 2) = ELEM(OppoEdge.form[0], OppoEdge.form[1], ExplodeOppoNode_num, STEINER_NOD, -1, -1, -1, setl->neig[mesh_process.Elem_Include_Node(*setl, ExplodeSameNode_num)]);
            ////    }
            ////}
        }
        // 当有4条边刺穿待恢复边界面时
        else if (setl->intersec_num - setl->vertex_num == 4)
        {
            std::cout << "There are 4 edges piercing the boundary surface to be restored!\n";
        }
        else
            std::cout << "Set initialized error, check the FindSet!\n", system("pause");
    }
    return;
}

void _BOUNDARY_RECOVERY::Setl_Generate_GridCell(_SU_MESH *su_mesh, std::vector<Setl> *set, FACE face_recovery)
{
    _MESH_PROCESS mesh_process;
    // 声明两个迭代器
    int elemNum_iter = -1;
    std::vector<FACE>::iterator face_iter;
    FACE *face_judge = nullptr;
    int *elemNum_judge = nullptr;
    int faceNum_cnt = 0;
    Setl setl;
    FACE face_tp;
    // 声明两个容器，分别存储待更新相邻信息的网格面与相对应的网格单元编号
    std::vector<int> elemNum_adjacent;
    std::vector<FACE> face_adjacent;
    // 声明一个数组，储存集上节点编号
    //int *set_nodeNum = (int *)malloc(sizeof(int) * (set->size() + 2));
    //int set_nodeNum_iter = 0;
    //*(set_nodeNum + set_nodeNum_iter++) = edge_recovery.form[0];
    // 集元包含5种类型，分别是0、1、2、3、4条边刺穿待恢复边界面
    for (std::vector<Setl>::iterator set_iter = set->begin(); set_iter != set->end(); ++set_iter)
    {
        setl = *set_iter;
        // 通过集元setl.intersec_num域与setl.vertex_num域的差值，判断当前集元类型，对不同类型进行不同形式的网格生成过程
        // 以下代码中若涉及到直接修改相邻信息的语句，其网格单元的顺序确定都是按照Decompose_Setl()函数中集元分解时确定的顺序来的
        // 首先会根据以上所述顺序直接更新集元内部网格单元之间的相邻信息和集元与普通网格单元之间的相邻信息，再更新集元与集元之间的相邻信息
        // 集元与集元之间相邻信息的更新会频繁利用到网格面的查找
        // 当没有边刺穿待恢复边界面时，该集元不需要再进行操作
        if (setl.intersec_num - setl.vertex_num == 0)
        {
            // 这种情况下不需要插入steiner点，储存当前边界面的分块信息
            face_tp = FACE(setl.vertex_nodeNum[0], setl.vertex_nodeNum[1], setl.vertex_nodeNum[2]);
            face_tp.Sort();
            if (std::find(su_mesh->boundary_face_Decom.begin(), su_mesh->boundary_face_Decom.end(), face_tp) == su_mesh->boundary_face_Decom.end())
                su_mesh->boundary_face_Decom.push_back(face_tp);
            continue;
        }
        // 当有1条边刺穿待恢复边界面时，该集元处理方式与边界边恢复中的单边型一致
        else if (setl.intersec_num - setl.vertex_num == 1)
        {
            // 首先将steiner点压入node容器，并储存该steiner点在网格中的节点编号
            // 先判断当前待压入的steiner点是否已被压入
            int steiner_node_num = -1;
            if (su_mesh->node.back() == NODE(*setl.pot))
                steiner_node_num = su_mesh->node_num - 1;
            else
            {
                su_mesh->node.push_back(NODE(*setl.pot));
                su_mesh->node.back().spac = mesh_process.get_aver_spac(su_mesh, su_mesh->elem.at(setl.elem_num));
                steiner_node_num = su_mesh->node_num++;
            }
            // 储存当前边界面的分块信息
            face_tp = FACE(setl.vertex_nodeNum[0], setl.vertex_nodeNum[1], steiner_node_num);
            face_tp.Sort();
            if (std::find(su_mesh->boundary_face_Decom.begin(), su_mesh->boundary_face_Decom.end(), face_tp) == su_mesh->boundary_face_Decom.end())
                su_mesh->boundary_face_Decom.push_back(face_tp);
            // 根据steiner点编号，更新集元分解生成的网格单元的节点信息
            (setl.Decom_elem + 0)->form[3] = steiner_node_num;
            (setl.Decom_elem + 1)->form[3] = steiner_node_num;
            su_mesh->node.at(steiner_node_num).elem = setl.elem_num;
            // 先更新所有能在该步骤下进行更新的相邻信息，再将新生成的两个网格单元插入elem容器
            // 得到新生成的网格单元使用的网格单元编号
            int setl_elem_num[] = {setl.elem_num, su_mesh->elem_num};
            // 更新这两个网格单元的相邻信息
            (setl.Decom_elem + 0)->neig[2] = setl_elem_num[1];
            (setl.Decom_elem + 1)->neig[2] = setl_elem_num[0];
            su_mesh->elem.at((setl.Decom_elem + 1)->neig[3]).neig[mesh_process.AdjacentElem_pos(su_mesh->elem.at((setl.Decom_elem + 1)->neig[3]), setl.elem_num)] = setl_elem_num[1];
            // 会有四个待判断的网格面
            faceNum_cnt = 4;
            face_judge = new FACE[4]{{(setl.Decom_elem + 0)->form[0], (setl.Decom_elem + 0)->form[2], (setl.Decom_elem + 0)->form[3]},
                                     {(setl.Decom_elem + 0)->form[1], (setl.Decom_elem + 0)->form[2], (setl.Decom_elem + 0)->form[3]},
                                     {(setl.Decom_elem + 1)->form[0], (setl.Decom_elem + 1)->form[2], (setl.Decom_elem + 1)->form[3]},
                                     {(setl.Decom_elem + 1)->form[1], (setl.Decom_elem + 1)->form[2], (setl.Decom_elem + 1)->form[3]}};
            // 这四个网格面相对应的网格单元编号如下
            elemNum_judge = new int[4]{setl_elem_num[0], setl_elem_num[0], setl_elem_num[1], setl_elem_num[1]};
            // 将新生成的两个网格单元在elem容器内替换掉集元所代表网格单元所在位置,或者压入elem
            (setl.Decom_elem + 0)->Sort();
            (setl.Decom_elem + 1)->Sort();
            su_mesh->elem.at(setl.elem_num) = *(setl.Decom_elem + 0);
            su_mesh->elem.push_back(*(setl.Decom_elem + 1));
            su_mesh->elem_num++;
            for (int i = 0; i < setl.Decom_elem_num; i++)
                mesh_process.Renew_NodeElem(su_mesh, setl_elem_num[i]);
        }
        // 当有2条边刺穿待恢复边界面时，该集元处理方式与边界边恢复中的邻边型一致
        else if (setl.intersec_num - setl.vertex_num == 2)
        {
            // 首先将steiner点压入node容器，并储存该steiner点在网格中的节点编号
            // 先判断当前待压入的steiner点是否已被压入
            int *steiner_node_num = new int[2]{-1, -1};
            for (int i = 0; i < 2; i++)
            {
                if (*(su_mesh->node.end() - 1) == NODE(*(setl.pot + i)))
                    *(steiner_node_num + i) = su_mesh->node_num - 1;
                else if (*(su_mesh->node.end() - 2) == NODE(*(setl.pot + i)))
                    *(steiner_node_num + i) = su_mesh->node_num - 2;
                else
                {
                    su_mesh->node.push_back(NODE(*(setl.pot + i)));
                    su_mesh->node.back().spac = mesh_process.get_aver_spac(su_mesh, su_mesh->elem.at(setl.elem_num));
                    *(steiner_node_num + i) = su_mesh->node_num++;
                }
            }
            // 储存当前边界面的分块信息
            face_tp = FACE(setl.vertex_nodeNum[0], *(steiner_node_num + 0), *(steiner_node_num + 1));
            face_tp.Sort();
            if (std::find(su_mesh->boundary_face_Decom.begin(), su_mesh->boundary_face_Decom.end(), face_tp) == su_mesh->boundary_face_Decom.end())
                su_mesh->boundary_face_Decom.push_back(face_tp);
            // 根据steiner点编号，更新集元分解生成的网格单元的节点信息，然后更新相邻信息，插入elem容器，最后储存待判断网格面
            // 首先得到待分解的两条网格边
            EDGE ExplodeEdge_1, ExplodeEdge_2;
            memcpy(ExplodeEdge_1.form, setl.intersec_edge, sizeof(ExplodeEdge_1.form));
            memcpy(ExplodeEdge_2.form, setl.intersec_edge + 2, sizeof(ExplodeEdge_2.form));
            // 邻边型的两条待分解边包含一个相同节点，记录该相同节点编号，再记录 在该两条待分解边组成的网格面中 该相同节点相对的网格边
            int ExplodeSameNode_num = -1;
            EDGE OppoEdge;
            for (int i = 0; i < 2; i++)
            {
                for (int j = 0; j < 2; j++)
                    if (ExplodeEdge_1.form[i] == ExplodeEdge_2.form[j])
                    {
                        ExplodeSameNode_num = ExplodeEdge_1.form[i];
                        OppoEdge.form[0] = ExplodeEdge_1.form[!i];
                        OppoEdge.form[1] = ExplodeEdge_2.form[!j];
                    }
            }
            // 首先得到与这两条待分解边组成的网格面在当前路径元内相对的节点编号位置
            FACE face_tp{ExplodeSameNode_num, OppoEdge.form[0], OppoEdge.form[1]};
            int face_pos = mesh_process.Face_Opposite_Node(setl, face_tp);
            // 保证正确性
            if (setl.Face_Decom_type[face_pos] != 'S' && setl.Face_Decom_type[face_pos] != 'Z')
                std::cout << "Bilateral setl's Decom_type_two_sides set error!\n", system("pause");
            // “S”型和“Z”型储存一致
            // 首先更新节点信息
            for (int i = 0; i < 2; i++)
                (setl.Decom_elem + i)->form[2] = *(steiner_node_num + 0), (setl.Decom_elem + i)->form[3] = *(steiner_node_num + 1);
            (setl.Decom_elem + 2)->form[3] = *(steiner_node_num + 0);
            su_mesh->node.at(*(steiner_node_num + 0)).elem = setl.elem_num, su_mesh->node.at(*(steiner_node_num + 1)).elem = setl.elem_num;
            // 得到新生成的网格单元使用的网格单元编号
            int setl_elem_num[] = {setl.elem_num, su_mesh->elem_num, su_mesh->elem_num + 1};
            // 更新相邻信息
            (setl.Decom_elem + 0)->neig[0] = setl_elem_num[1];
            (setl.Decom_elem + 1)->neig[0] = setl_elem_num[0], (setl.Decom_elem + 1)->neig[3] = setl_elem_num[2];
            (setl.Decom_elem + 2)->neig[0] = setl_elem_num[1];
            su_mesh->elem.at((setl.Decom_elem + 2)->neig[3]).neig[mesh_process.AdjacentElem_pos(su_mesh->elem.at((setl.Decom_elem + 2)->neig[3]), setl.elem_num)] = setl_elem_num[2];
            // 邻边型会有七个待判断的网格面
            faceNum_cnt = 7;
            face_judge = new FACE[7];   // 储存七个网格面
            elemNum_judge = new int[7]; // 储存这七个网格面相对应的网格单元编号
            *(face_judge + 0) = FACE{(setl.Decom_elem + 0)->form[0], (setl.Decom_elem + 0)->form[1], (setl.Decom_elem + 0)->form[2]};
            *(elemNum_judge + 0) = setl_elem_num[0];
            for (int i = 0; i < 3; i++)
            {
                *(face_judge + 2 * i + 1) = FACE{(setl.Decom_elem + i)->form[0], (setl.Decom_elem + i)->form[1], (setl.Decom_elem + i)->form[3]};
                *(face_judge + 2 * i + 2) = FACE{(setl.Decom_elem + i)->form[0], (setl.Decom_elem + i)->form[2], (setl.Decom_elem + i)->form[3]};
                *(elemNum_judge + 2 * i + 1) = setl_elem_num[i];
                *(elemNum_judge + 2 * i + 2) = setl_elem_num[i];
                (setl.Decom_elem + i)->Sort();
            }
            // 将新生成的三个网格单元在elem容器内替换掉集元所代表网格单元所在位置,或者压入elem
            su_mesh->elem.at(setl.elem_num) = *(setl.Decom_elem + 0);
            for (int i = 1; i < 3; i++)
            {
                su_mesh->elem.push_back(*(setl.Decom_elem + i));
                su_mesh->elem_num++;
            }
            for (int i = 0; i < setl.Decom_elem_num; i++)
                mesh_process.Renew_NodeElem(su_mesh, setl_elem_num[i]);
            delete[] steiner_node_num;
        }
        // 当有3条边刺穿待恢复边界面时
        else if (setl.intersec_num - setl.vertex_num == 3)
        {
        }
        // 当有4条边刺穿待恢复边界面时
        else if (setl.intersec_num - setl.vertex_num == 4)
        {
        }
        else
            std::cout << "Set initialized error, check the FindSet!\n", system("pause");
        // 更新集元与集元之间的相邻信息
        // 在face_adjacent容器内查找这些网格面，若不存在则直接压入容器并将这个网格面代表的网格单元编号一并压入容器elemNum_adjacent，若存在则取出相对于的网格面与网格单元编号，并进行相邻信息更新
        int elemNum_tp = -1;
        // 保证数据有效
        if (face_judge == nullptr || elemNum_judge == nullptr)
            std::cout << "Setl generate grid cell error!\n", system("pause");
        else
            for (int i = 0; i < faceNum_cnt; i++)
            {
                face_judge[i].Sort();
                if ((face_iter = std::find(face_adjacent.begin(), face_adjacent.end(), face_judge[i])) == face_adjacent.end())
                {
                    face_adjacent.push_back(face_judge[i]);
                    elemNum_adjacent.push_back(elemNum_judge[i]);
                }
                else
                {
                    elemNum_iter = std::distance(face_adjacent.begin(), face_iter);
                    if (elemNum_iter < int(elemNum_adjacent.size()))
                        elemNum_tp = elemNum_adjacent.at(elemNum_iter);
                    else
                        std::cout << "Setl generate grid cell error, the elemNum_iter is out of bounds!\n", system("pause");
                    // 从容器内删除这两个值
                    face_adjacent.erase(face_iter);
                    elemNum_adjacent.erase(elemNum_adjacent.begin() + elemNum_iter);
                    // 修改包含该网格面的两个网格单元的相邻信息
                    su_mesh->elem.at(elemNum_judge[i]).neig[mesh_process.Face_Opposite_Node(su_mesh->elem.at(elemNum_judge[i]), face_judge[i])] = elemNum_tp;
                    su_mesh->elem.at(elemNum_tp).neig[mesh_process.Face_Opposite_Node(su_mesh->elem.at(elemNum_tp), face_judge[i])] = elemNum_judge[i];
                }
            }
        elemNum_iter = -1;
        delete[] face_judge;
        face_judge = nullptr;
        delete[] elemNum_judge;
        elemNum_judge = nullptr;
        faceNum_cnt = 0;
    }
    if (!face_adjacent.empty())
        std::cout << "When Pathl_Generate_GridCell, the pathl decompose error, some adjacent face were not found successfully!\n", system("pause");
    setl.Decom_elem = nullptr; // 避免析构函数错误删除内存空间

    //// 更新集节点信息，最后一个节点
    //*(set_nodeNum + set_nodeNum_iter++) = edge_recovery.form[1];
    //NODE node_tp;
    //int nodeNum_tp;
    //int elemNum_neig_tp[2] = {-1, -1};
    //EDGE edge_tp;
    //ELEM elem_tp;
    //bool merge_judge = false; // 判断上次循环是否进行了合并操作
    //std::vector<int> elemNum_IncludeNode;
    //std::vector<int> elemNum_IncludeEdge;
    //std::vector<int> elemNum_wait_delete;
    //int *nodeNum_wait_delete = new int[set_nodeNum_iter]; // 储存合并结束后需要删除的节点编号
    //int nodeNum_wait_delete_cnt = 0;
    //// 利用集节点信息，判断这些新插入的steiner点间距离，若过短，则进行合并操作
    //for (int i = 1; i < set_nodeNum_iter - 2; i++)
    //{
    //    // 取出待判断边
    //    if (merge_judge)
    //        edge_tp = EDGE(nodeNum_tp, *(set_nodeNum + i + 1));
    //    else
    //        edge_tp = EDGE(*(set_nodeNum + i), *(set_nodeNum + i + 1));
    //    merge_judge = false;
    //    std::vector<int>().swap(elemNum_IncludeEdge);
    //    mesh_process.FindRing(su_mesh, edge_tp, &elemNum_IncludeEdge, "fast");
    //    if (elemNum_IncludeEdge.empty())
    //        std::cout << "Setl generate gridCell merge fail, elemNum_IncludeEdge search error!\n", system("pause");
    //    // 通过该边长度判断是否需要合并，低于模型最短边界边的一定倍数值时需要合并
    //    if (data_process.get_dist(su_mesh->node.at(edge_tp.form[0]).pos, su_mesh->node.at(edge_tp.form[1]).pos) <= su_mesh->shortest_border_edge * Max_steiner_point_internal)
    //    {
    //        node_tp = (su_mesh->node.at(edge_tp.form[0]) + su_mesh->node.at(edge_tp.form[1])) * 0.5;
    //        nodeNum_tp = edge_tp.form[0];
    //        *(nodeNum_wait_delete + nodeNum_wait_delete_cnt++) = edge_tp.form[1]; // 待删除节点编号，节点留在整个流程结束后再删除
    //        // elemNum_IncludeEdge内的网格单元在节点合并结束后都会成为无效单元，需要进行删除操作
    //        // 先将这些网格单元替换到elem容器末尾，以便后续删除操作，并记录下这些网格单元在替换后的网格单元编号
    //        std::vector<int>().swap(elemNum_wait_delete);
    //        int cnt_elem = 0; // 记录待删除网格单元的个数
    //        std::sort(elemNum_IncludeEdge.begin(), elemNum_IncludeEdge.end());
    //        for (int j = int(elemNum_IncludeEdge.size()) - 1; j >= 0; j--)
    //        {
    //            // 将该网格单元与elem容器的最后一个有效网格单元交换位置
    //            // 如果该网格单元就位于elem容器的最后一个有效网格单元位置，则不交换
    //            if (elemNum_IncludeEdge.at(j) == su_mesh->elem.size() - 1 - cnt_elem)
    //            {
    //                cnt_elem++;
    //                elemNum_wait_delete.push_back(elemNum_IncludeEdge.at(j));
    //                continue;
    //            }
    //            ReplaceElem_two(su_mesh, elemNum_IncludeEdge.at(j), su_mesh->elem_num - 1 - cnt_elem);
    //            elemNum_wait_delete.push_back(su_mesh->elem_num - 1 - cnt_elem);
    //            cnt_elem++;
    //        }
    //        // 修改合并后的节点信息
    //        su_mesh->node.at(nodeNum_tp) = node_tp;
    //        // 储存待修改节点信息的网格单元
    //        std::vector<int>().swap(elemNum_IncludeNode);
    //        mesh_process.FindBall_fast(su_mesh, edge_tp.form[1], &elemNum_IncludeNode);
    //        // 修改节点合并后相关网格单元的相邻信息
    //        for (std::vector<int>::iterator iter = elemNum_wait_delete.begin(); iter != elemNum_wait_delete.end(); ++iter)
    //        {
    //            elem_tp = su_mesh->elem.at(*iter);
    //            elemNum_neig_tp[0] = elem_tp.neig[mesh_process.ELEM_Include_Node(elem_tp, edge_tp.form[1])];
    //            elemNum_neig_tp[1] = elem_tp.neig[mesh_process.ELEM_Include_Node(elem_tp, edge_tp.form[0])];
    //            su_mesh->elem.at(elemNum_neig_tp[0]).neig[mesh_process.AdjacentElem_pos(su_mesh->elem.at(elemNum_neig_tp[0]), *iter)] = elemNum_neig_tp[1];
    //            su_mesh->elem.at(elemNum_neig_tp[1]).neig[mesh_process.AdjacentElem_pos(su_mesh->elem.at(elemNum_neig_tp[1]), *iter)] = elemNum_neig_tp[0];
    //            mesh_process.Renew_NodeElem(su_mesh, elemNum_neig_tp[0]);
    //            mesh_process.Renew_NodeElem(su_mesh, elemNum_neig_tp[1]);
    //        }
    //        // 删除相关网格单元
    //        for (int k = 0; k < cnt_elem; k++)
    //            su_mesh->elem.pop_back(), su_mesh->elem_num--;
    //        // 修改网格单元节点编号信息
    //        for (std::vector<int>::iterator iter = elemNum_IncludeNode.begin(); iter != elemNum_IncludeNode.end(); ++iter)
    //            if (*iter < su_mesh->elem_num)
    //                su_mesh->elem.at(*iter).form[mesh_process.ELEM_Include_Node(su_mesh->elem.at(*iter), edge_tp.form[1])] = nodeNum_tp, su_mesh->elem.at(*iter).Sort();
    //        merge_judge = true;
    //    }
    //}
    //// 删除无效节点，先将其替换到node容器末尾，再进行删除操作
    //std::sort(nodeNum_wait_delete, nodeNum_wait_delete + nodeNum_wait_delete_cnt, std::greater<>());
    //for (int i = 0; i < nodeNum_wait_delete_cnt; i++)
    //{
    //    nodeNum_tp = su_mesh->node_num - 1;
    //    // 如果本来就在末尾，则直接删除
    //    if (*(nodeNum_wait_delete + i) == nodeNum_tp)
    //    {
    //        su_mesh->node.pop_back();
    //        su_mesh->node_num--;
    //        continue;
    //    }
    //    std::vector<int>().swap(elemNum_IncludeNode);
    //    mesh_process.FindBall_fast(su_mesh, nodeNum_tp, &elemNum_IncludeNode); // 查找包含node容器末尾节点的所有网格单元编号
    //    // 修改elem容器内值
    //    for (std::vector<int>::iterator iter = elemNum_IncludeNode.begin(); iter != elemNum_IncludeNode.end(); ++iter)
    //        su_mesh->elem.at(*iter).form[mesh_process.Elem_Include_Node(su_mesh->elem.at(*iter), nodeNum_tp)] = *(nodeNum_wait_delete + i), su_mesh->elem.at(*iter).Sort();
    //    // 替换节点位置
    //    std::swap(su_mesh->node.at(nodeNum_tp), su_mesh->node.at(*(nodeNum_wait_delete + i)));
    //    su_mesh->node.pop_back();
    //    su_mesh->node_num--;
    //}
    //free(set_nodeNum);
    //set_nodeNum = nullptr;
    //delete[] nodeNum_wait_delete;
    return;
}

void _BOUNDARY_RECOVERY::Recovery_Boundary_face(_SU_MESH *su_mesh, FACE face_recovery)
{
    _MESH_PROCESS mesh_process;
    _QUALITY quality;
    // 首先查找待恢复边界面的集（set），记录其与集元（setl）相交点及其数量
    std::vector<Setl> set = FindSet(su_mesh, face_recovery);
    for (std::vector<Setl>::iterator setl = set.begin(); setl != set.end(); ++setl)
        std::cout << setl->intersec_num - setl->vertex_num << ' ';
    std::cout << '\n';
    // 如果集中只有三个集元，则可以通过Swap32实现约束边界恢复
    if (set.size() == 3)
    {
        // 查找这三个集元的相同相邻边
        EDGE edge_tp = mesh_process.elem_AdjacentEdge(*(set.begin() + 0), *(set.begin() + 1), *(set.begin() + 2));
        quality.Face_Transform_32(su_mesh, edge_tp);
    }
    else
    {
        // 遍历每个集元，分别进行分解和生成网格单元，同时确保相邻信息的准确性
        // 对集元进行分解操作
        Decompose_Setl(&set);
        // 依据集中各个集元类型，将分解后的网格压入elem容器，形成集元的完整分解生成过程
        Setl_Generate_GridCell(su_mesh, &set, face_recovery);
    }
    //mesh_process.Judge_the_validity_of_information(su_mesh);
    return;
}

bool _BOUNDARY_RECOVERY::Split_Edge_Steiner_Point(_SU_MESH *su_mesh, int SteinerNodeNum_wait_split)
{
    _MESH_PROCESS mesh_process;
    _DATA_PROCESS data_process;
    _CAVITY cavity;

    // 首先查找节点的Ball
    std::vector<int> elemNum_IncludeNode;
    mesh_process.FindBall_fast(su_mesh, SteinerNodeNum_wait_split, &elemNum_IncludeNode);
    if (elemNum_IncludeNode.empty()) // 如果查找失败，返回false
        return false;

    // 然后在 被分解的边界面片(su_mesh->boundary_face_Decom) 内查找包含当前Steiner点的边界面
    std::vector<int> face_num;
    face_num = mesh_process.Find_face_from_FaceGroup(su_mesh->boundary_face_Decom, SteinerNodeNum_wait_split);
    if (face_num.empty()) // 如果查找失败，返回false
        return false;
    // 在su_mesh->boundary_face_Decom内删除上述查找到的边界面并储存
    std::vector<FACE> face_wait_process;
    std::sort(face_num.begin(), face_num.end(), std::greater<int>()); // 逆序排序
    for (std::vector<int>::iterator iter = face_num.begin(); iter != face_num.end(); ++iter)
    {
        face_wait_process.push_back(su_mesh->boundary_face_Decom.at(*iter));
        su_mesh->boundary_face_Decom.erase(su_mesh->boundary_face_Decom.begin() + *iter);
    }
    // 进一步分析face_wait_process内边界面，将这些边界面视为一个环绕SteinerNodeNum_wait_split的环，储存其边界边（即这些边界面内不包含SteinerNodeNum_wait_split点的边界边）
    std::vector<EDGE> edge_surround_Steiner;
    for (std::vector<FACE>::iterator iter = face_wait_process.begin(); iter != face_wait_process.end(); ++iter)
    {
        for (int i = 0; i < 2; i++)
            for (int j = i + 1; j < 3; j++)
                if (iter->form[i] == SteinerNodeNum_wait_split || iter->form[j] == SteinerNodeNum_wait_split)
                    continue;
                else
                    edge_surround_Steiner.push_back(EDGE(iter->form[i], iter->form[j]));
    }

    // 然后在 被分解的边界边集合(su_mesh->boundary_edge_Decom) 内查找包含当前Steiner点的边界边
    std::vector<int> edge_num;
    edge_num = mesh_process.Find_edge_from_EdgeSet(su_mesh->boundary_edge_Decom, SteinerNodeNum_wait_split);
    if (edge_num.size() != 2) // 如果查找失败，返回false，此时face_num内有且只有两个值
        return false;
    // 在su_mesh->boundary_edge_Decom内删除上述查找到的边界边并储存
    std::vector<EDGE> edge_wait_process;
    std::sort(edge_num.begin(), edge_num.end(), std::greater<int>()); // 逆序排序
    for (std::vector<int>::iterator iter = edge_num.begin(); iter != edge_num.end(); ++iter)
    {
        edge_wait_process.push_back(su_mesh->boundary_edge_Decom.at(*iter));
        su_mesh->boundary_edge_Decom.erase(su_mesh->boundary_edge_Decom.begin() + *iter);
    }
    // 此时可以储存当前SteinerNodeNum_wait_split点分解后，还原的目标边界边
    EDGE target_BoundaryEdge;
    int cnt = 0;
    for (std::vector<EDGE>::iterator iter = edge_wait_process.begin(); iter != edge_wait_process.end(); ++iter)
    {
        for (int i = 0; i < 2; i++)
            if (iter->form[i] != SteinerNodeNum_wait_split)
                target_BoundaryEdge.form[cnt++] = iter->form[i];
    }
    target_BoundaryEdge.Sort();

    // 重连环绕SteinerNodeNum_wait_split的环，使target_BoundaryEdge存在，并生成一组新的边界面，这组边界面即是当前SteinerNodeNum_wait_split点分解后，还原的 目标边界面片的 更大的分块边界面片
    std::vector<FACE> target_BoundaryFace;
    int nodeNum_tp = target_BoundaryEdge.form[0]; // 取target_BoundaryEdge的任意一个节点，连接edge_surround_Steiner内不包含该节点的网格边，形成新的网格面
    for (std::vector<EDGE>::iterator iter = edge_surround_Steiner.begin(); iter != edge_surround_Steiner.end(); ++iter)
    {
        if (iter->form[0] != nodeNum_tp && iter->form[1] != nodeNum_tp)
        {
            target_BoundaryFace.push_back(FACE(nodeNum_tp, iter->form[0], iter->form[1]));
            target_BoundaryFace.back().Sort();
        }
    }

    // 利用face_wait_process信息和染色法，将elemNum_IncludeNode分解为两个Ball
    std::vector<int> elemNum_IncludeSplitNode[2];
    int *elem_symbol = new int[int(elemNum_IncludeNode.size())];
    elem_symbol[0] = 1;
    std::deque<int> elemNum_wait;
    std::vector<int> elemNum_succ;
    elemNum_wait.push_back(0);                         // 这个编号不是节点编号，指的是elemNum_IncludeNode容器元素位置
    elemNum_succ.push_back(elemNum_IncludeNode.at(0)); // 这个编号是节点编号
    int Num_tp = -1, Num_iter = -1;
    int elemNeigNum_tp = -1;
    ELEM elem_tp;
    FACE face_tp;
    std::vector<int>::iterator elemNum_iter;
    while (elemNum_succ.size() < elemNum_IncludeNode.size())
    {
        Num_tp = elemNum_wait.front();
        elem_tp = su_mesh->elem.at(elemNum_IncludeNode.at(Num_tp));
        elemNum_wait.pop_front();
        for (int i = 0; i < DIM + 1; i++)
        {
            elemNeigNum_tp = elem_tp.neig[i];
            if (elemNeigNum_tp == -1)
                continue;
            if (std::find(elemNum_succ.begin(), elemNum_succ.end(), elemNeigNum_tp) != elemNum_succ.end())
                continue;
            if ((elemNum_iter = std::find(elemNum_IncludeNode.begin(), elemNum_IncludeNode.end(), elemNeigNum_tp)) != elemNum_IncludeNode.end())
            {
                face_tp = mesh_process.Node_Opposite_Face(elem_tp, elem_tp.form[i]);
                Num_iter = int(std::distance(elemNum_IncludeNode.begin(), elemNum_iter));
                if (std::find(face_wait_process.begin(), face_wait_process.end(), face_tp) == face_wait_process.end())
                    elem_symbol[Num_iter] = elem_symbol[Num_tp];
                else
                    elem_symbol[Num_iter] = -elem_symbol[Num_tp];
                elemNum_wait.push_back(Num_iter);
                elemNum_succ.push_back(elemNum_IncludeNode.at(Num_iter));
            }
        }
    }
    for (int i = 0; i < int(elemNum_IncludeNode.size()); i++)
    {
        if (*(elem_symbol + i) == 1)
            elemNum_IncludeSplitNode[0].push_back(elemNum_IncludeNode.at(i));
        else
            elemNum_IncludeSplitNode[1].push_back(elemNum_IncludeNode.at(i));
    }

    // 分别储存两个Ball的边界面（删除face_wait_process中边界面并增加target_BoundaryFace中边界面），生成两个完整球面，并记录两个Ball原始体积，与其相邻信息
    std::vector<FACE> face_IncludeSplitNode[2];
    std::vector<FACE>::iterator face_iter;
    std::vector<int> elemNum_adjacent; // 存储两个Ball的外边界中待更新相邻信息的网格面与相对应的网格单元编号
    std::vector<FACE> face_adjacent;
    double original_volume[2] = {0, 0};
    int elemAdjacent_iter = -1;
    for (int i = 0; i < 2; i++)
    {
        for (std::vector<int>::iterator iter = elemNum_IncludeSplitNode[i].begin(); iter != elemNum_IncludeSplitNode[i].end(); ++iter)
        {
            elem_tp = su_mesh->elem.at(*iter);
            original_volume[i] += abs(data_process.tetrahedral_volume(su_mesh->node.at(elem_tp.form[0]), su_mesh->node.at(elem_tp.form[1]), su_mesh->node.at(elem_tp.form[2]), su_mesh->node.at(elem_tp.form[3])));
            for (int j = 0; j < 4; j++)
            {
                face_tp = mesh_process.Node_Opposite_Face(elem_tp, elem_tp.form[j]);
                if (mesh_process.FACE_Include_Node(face_tp, SteinerNodeNum_wait_split) != -1)
                    continue;
                face_IncludeSplitNode[i].push_back(face_tp);
                elemNum_adjacent.push_back(elem_tp.neig[j]);
                face_adjacent.push_back(face_tp);
            }
        }
        face_IncludeSplitNode[i].insert(face_IncludeSplitNode[i].end(), target_BoundaryFace.begin(), target_BoundaryFace.end());
    }

    // 初始化两个Ball，并且记录网格单元编号
    // 此处可以直接用 elemNum_IncludeNode
    cavity.Initialize_Cavity(su_mesh, elemNum_IncludeNode);

    // 分别对两个Ball进行插点重连操作，其外边界面已储存在face_IncludeSplitNode中，利用插点后的网格体积之和来判断当前插入点是否合法
    // 待插入点初始化为Ball重心位置，若非法，则沿一定比例向SteinerNodeNum_wait_split点移动并且在此判断，直至合法
    double volume = 0.0;
    int elemNum_tp = -1, elemGenNum_tp = -1;
    nodeNum_tp = -1;
    NODE node_steiner = su_mesh->node.at(SteinerNodeNum_wait_split);
    NODE node_insert[2];
    double x_value = 0, x_cnt = 0, y_value = 0, y_cnt = 0, z_value = 0, z_cnt = 0; // 定义一系列变量，储存x、y与z方向总坐标之和与总点数
    double vector[3] = {0.0, 0.0, 0.0};
    for (int i = 0; i < 2; i++)
    {
        node_insert[i].spac = node_steiner.spac;
        x_value = 0, x_cnt = 0, y_value = 0, y_cnt = 0, z_value = 0, z_cnt = 0;
        // 求Ball重心位置
        for (std::vector<FACE>::iterator iter = face_IncludeSplitNode[i].begin(); iter != face_IncludeSplitNode[i].end(); ++iter)
            for (int j = 0; j < 3; j++)
            {
                x_value += su_mesh->node.at(iter->form[j]).pos[0];
                x_cnt++;
                y_value += su_mesh->node.at(iter->form[j]).pos[1];
                y_cnt++;
                z_value += su_mesh->node.at(iter->form[j]).pos[2];
                z_cnt++;
            }
        node_insert[i].pos[0] = x_value / x_cnt, node_insert[i].pos[1] = y_value / y_cnt, node_insert[i].pos[2] = z_value / z_cnt;
        vector[0] = node_steiner.pos[0] - node_insert[i].pos[0], vector[1] = node_steiner.pos[1] - node_insert[i].pos[1], vector[2] = node_steiner.pos[2] - node_insert[i].pos[2];
        cnt = 0;
        volume = 0;
        // 查找合法节点位置
        while (abs(volume - original_volume[i]) >= volume_error)
        {
            volume = 0;
            if (cnt > 0)
            {
                node_insert[i].pos[0] += vector[0] * 0.5, node_insert[i].pos[1] += vector[1] * 0.5, node_insert[i].pos[2] += vector[2] * 0.5;
                vector[0] = node_steiner.pos[0] - node_insert[i].pos[0], vector[1] = node_steiner.pos[1] - node_insert[i].pos[1], vector[2] = node_steiner.pos[2] - node_insert[i].pos[2];
            }
            // 求插入点后拟生成的网格单元体积总和
            for (std::vector<FACE>::iterator iter = face_IncludeSplitNode[i].begin(); iter != face_IncludeSplitNode[i].end(); ++iter)
                volume += abs(data_process.tetrahedral_volume(su_mesh->node.at(iter->form[0]), su_mesh->node.at(iter->form[1]), su_mesh->node.at(iter->form[2]), node_insert[i]));
            cnt++;
            if (cnt > 13)
                std::cout << SteinerNodeNum_wait_split << " Edge steiner point split error!\n", system("pause");
        }
        // 将其插入node容器
        su_mesh->node.push_back(node_insert[i]);
        nodeNum_tp = su_mesh->node_num++;
        // 开始生成网格单元
        for (std::vector<FACE>::iterator iter = face_IncludeSplitNode[i].begin(); iter != face_IncludeSplitNode[i].end(); ++iter)
        {
            elem_tp = ELEM(iter->form[0], iter->form[1], iter->form[2], nodeNum_tp);
            elem_tp.Sort();
            if (elemNum_IncludeNode.empty())
            {
                su_mesh->elem.push_back(elem_tp);
                elemGenNum_tp = su_mesh->elem_num++;
            }
            else
            {
                elemGenNum_tp = elemNum_IncludeNode.back();
                elemNum_IncludeNode.pop_back();
                su_mesh->elem.at(elemGenNum_tp) = elem_tp;
            }
            mesh_process.Renew_NodeElem(su_mesh, elemGenNum_tp);
            for (int m = 0; m < 2; m++)
                for (int n = m + 1; n < 3; n++)
                    for (int k = n + 1; k < 4; k++)
                    {
                        face_tp = FACE(elem_tp.form[m], elem_tp.form[n], elem_tp.form[k]);
                        if ((face_iter = std::find(face_adjacent.begin(), face_adjacent.end(), face_tp)) == face_adjacent.end())
                        {
                            face_adjacent.push_back(face_tp);
                            elemNum_adjacent.push_back(elemGenNum_tp);
                        }
                        else
                        {
                            elemAdjacent_iter = std::distance(face_adjacent.begin(), face_iter);
                            if (elemAdjacent_iter < int(elemNum_adjacent.size()))
                                elemNum_tp = elemNum_adjacent.at(elemAdjacent_iter);
                            else
                                std::cout << "Split_Edge_Steiner_Point, the ball generate error!\n", system("pause");
                            // 从容器内删除这两个值
                            face_adjacent.erase(face_iter);
                            elemNum_adjacent.erase(elemNum_adjacent.begin() + elemAdjacent_iter);
                            // 修改包含该网格面的两个网格单元的相邻信息
                            su_mesh->elem.at(elemGenNum_tp).neig[mesh_process.Face_Opposite_Node(su_mesh->elem.at(elemGenNum_tp), face_tp)] = elemNum_tp;
                            su_mesh->elem.at(elemNum_tp).neig[mesh_process.Face_Opposite_Node(su_mesh->elem.at(elemNum_tp), face_tp)] = elemGenNum_tp;
                        }
                    }
        }
    }

    // 检查生成的目标边界边与目标边界面是否是原始网格边界边与边界面，若不是则继续压入相应容器
    if (std::find(su_mesh->boundary_edge.begin(), su_mesh->boundary_edge.end(), target_BoundaryEdge) == su_mesh->boundary_edge.end())
        su_mesh->boundary_edge_Decom.push_back(target_BoundaryEdge);
    for (std::vector<FACE>::iterator iter = target_BoundaryFace.begin(); iter != target_BoundaryFace.end(); iter++)
        if (std::find(su_mesh->boundary_face.begin(), su_mesh->boundary_face.end(), *iter) == su_mesh->boundary_face.end())
            su_mesh->boundary_face_Decom.push_back(*iter);

    delete[] elem_symbol;
    elem_symbol = nullptr;
    return true;
}

void _BOUNDARY_RECOVERY::Insert_Steiner_Points(_SU_MESH *su_mesh)
{
    _MESH_PROCESS mesh_process;
    int Steiner_insert_symbol = su_mesh->node_num; // 判断每次边界边恢复或者边界面恢复时是否插入了Steiner点
    // 首先恢复边界边，查找所有不在当前三角化内的边界边
    su_mesh->nodeNum_before_edge_recovery = su_mesh->node_num; // 储存边界边恢复前节点数目
    std::vector<EDGE> edge_wait_recovery;                      // 储存待恢复的边界边
    std::vector<int> elemNum_IncludeEdge;
    for (std::vector<EDGE>::iterator iter = su_mesh->boundary_edge.begin(); iter != su_mesh->boundary_edge.end(); ++iter)
    {
        std::vector<int>().swap(elemNum_IncludeEdge);
        mesh_process.FindRing(su_mesh, *iter, &elemNum_IncludeEdge, "fast");
        if (elemNum_IncludeEdge.empty())
            edge_wait_recovery.push_back(*iter);
    }
    // 一个个恢复边界边
    for (std::vector<EDGE>::iterator iter = edge_wait_recovery.begin(); iter != edge_wait_recovery.end(); ++iter)
    {
        Recovery_Boundary_edge(su_mesh, *iter);
        for (int i = Steiner_insert_symbol; i < su_mesh->node_num; i++)
        {
            su_mesh->node.at(static_cast<std::vector<NODE, std::allocator<NODE>>::size_type>(i)).Steiner_symbol = 1;
            su_mesh->node.at(static_cast<std::vector<NODE, std::allocator<NODE>>::size_type>(i)).Steiner_source[0] = iter->form[0];
            su_mesh->node.at(static_cast<std::vector<NODE, std::allocator<NODE>>::size_type>(i)).Steiner_source[1] = iter->form[1];
        }
        Steiner_insert_symbol = su_mesh->node_num;
    }
    su_mesh->nodeNum_after_edge_recovery = su_mesh->node_num;  // 储存边界边恢复后节点数目
    su_mesh->nodeNum_before_face_recovery = su_mesh->node_num; // 储存边界面恢复前节点数目
    // 再恢复边界面
    std::vector<FACE> face_wait_recovery; // 储存待恢复的边界面
    std::vector<int> elemNum_IncludeFace;
    for (std::vector<FACE>::iterator iter = su_mesh->boundary_face.begin(); iter != su_mesh->boundary_face.end(); ++iter)
    {
        std::vector<int>().swap(elemNum_IncludeFace);
        mesh_process.FindAwl(su_mesh, *iter, &elemNum_IncludeFace, "fast");
        if (elemNum_IncludeFace.empty())
            face_wait_recovery.push_back(*iter);
    }
    for (std::vector<FACE>::iterator iter = face_wait_recovery.begin(); iter != face_wait_recovery.end(); ++iter)
    {
        Recovery_Boundary_face(su_mesh, *iter);
        for (int i = Steiner_insert_symbol; i < su_mesh->node_num; i++)
        {
            su_mesh->node.at(static_cast<std::vector<NODE, std::allocator<NODE>>::size_type>(i)).Steiner_symbol = 2;
            su_mesh->node.at(static_cast<std::vector<NODE, std::allocator<NODE>>::size_type>(i)).Steiner_source[0] = iter->form[0];
            su_mesh->node.at(static_cast<std::vector<NODE, std::allocator<NODE>>::size_type>(i)).Steiner_source[1] = iter->form[1];
            su_mesh->node.at(static_cast<std::vector<NODE, std::allocator<NODE>>::size_type>(i)).Steiner_source[2] = iter->form[2];
        }
        Steiner_insert_symbol = su_mesh->node_num;
    }
    su_mesh->nodeNum_after_face_recovery = su_mesh->node_num; // 储存边界面恢复后节点数目

    // 储存边界恢复结束后节点数目
    su_mesh->boundary_recovery_node_num = su_mesh->node_num;

    return;
}

void _BOUNDARY_RECOVERY::Split_Steiner_Points(_SU_MESH *su_mesh)
{
    _MESH_PROCESS mesh_process;
    // 先分解移动边界边上的Steiner点
    for (int i = su_mesh->nodeNum_before_edge_recovery; i < su_mesh->nodeNum_after_edge_recovery; i++)
        if (!Split_Edge_Steiner_Point(su_mesh, i))
        {
            std::cout << "Steiner point split error, check the program!\n";
            mesh_process.Judge_the_validity_of_information(su_mesh);
        }
    // 利用节点找分段边界边找分块边界面，分段边界面的节点形成一个环，球内剩余节点为外部节点，再进行网格分解

    // 删除边界边上无效Steiner点，先将其替换到node容器末尾，再进行删除操作
    int nodeNum_tp = -1;
    std::vector<int> elemNum_IncludeNode;
    for (int i = su_mesh->nodeNum_after_edge_recovery - 1; i >= su_mesh->nodeNum_before_edge_recovery; i--)
    {
        nodeNum_tp = su_mesh->node_num - 1;
        // 如果本来就在末尾，则直接删除
        if (i == nodeNum_tp)
        {
            su_mesh->node.pop_back();
            su_mesh->node_num--;
            continue;
        }
        std::vector<int>().swap(elemNum_IncludeNode);
        mesh_process.FindBall_fast(su_mesh, nodeNum_tp, &elemNum_IncludeNode); // 查找包含node容器末尾节点的所有网格单元编号
        // 修改elem容器内值
        for (std::vector<int>::iterator iter = elemNum_IncludeNode.begin(); iter != elemNum_IncludeNode.end(); ++iter)
            su_mesh->elem.at(*iter).form[mesh_process.Elem_Include_Node(su_mesh->elem.at(*iter), nodeNum_tp)] = i, su_mesh->elem.at(*iter).Sort();
        // 替换节点位置
        std::swap(su_mesh->node.at(nodeNum_tp), su_mesh->node.at(i));
        // 避免初始边界点被修改
        if (nodeNum_tp < su_mesh->InitNode_num + 8 || i < su_mesh->InitNode_num + 8)
            std::cout << "InitNode being change!\n";
        su_mesh->node.pop_back();
        su_mesh->node_num--;
    }
    //mesh_process.Judge_the_validity_of_information(su_mesh);
    return;
}

void _BOUNDARY_RECOVERY::Restore_Judgment(_SU_MESH *su_mesh)
{
    _MESH_PROCESS mesh_process;
    // 检查所有不在当前三角化内的边界边
    std::vector<int> elemNum;
    for (std::vector<EDGE>::iterator iter = su_mesh->boundary_edge.begin(); iter != su_mesh->boundary_edge.end(); ++iter)
    {
        std::vector<int>().swap(elemNum);
        mesh_process.FindRing(su_mesh, *iter, &elemNum, "fast");
        if (elemNum.empty())
            std::cout << "Boundart edge formed by " << iter->form[0] << ' ' << iter->form[1] << " is not recovered!\n";
    }
    // 再检查边界面
    for (std::vector<FACE>::iterator iter = su_mesh->boundary_face.begin(); iter != su_mesh->boundary_face.end(); ++iter)
    {
        std::vector<int>().swap(elemNum);
        mesh_process.FindAwl(su_mesh, *iter, &elemNum, "fast");
        if (elemNum.empty())
            std::cout << "Boundart face formed by " << iter->form[0] << ' ' << iter->form[1] << ' ' << iter->form[2] << " is not recovered!\n";
    }
    return;
}

bool _BOUNDARY_RECOVERY::operator==(const _BOUNDARY_RECOVERY &other) const
{
    return false;
}
