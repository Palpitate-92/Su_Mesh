#include "Su_Mesh.h"

std::vector<int> _BOUNDARY_RECOVERY::External_Elem_Lookup(_SU_MESH *su_mesh)
{
    // 采用着色法来确定外部单元，若两个网格单元相邻网格面为边界面，则其标记相反
    std::vector<int> elemNum_External;                                           // 声明一个容器，用来储存外部网格单元编号
    int *external_elem_judge = (int *)malloc((su_mesh->elem_num) * sizeof(int)); // 作为每个网格单元的标记数组
    memset(external_elem_judge, 0, (su_mesh->elem_num) * sizeof(int));           // 1代表非外部单元，-1代表外部单元,0代表未被判断
    int ini_elemNum = 0;
    // 该方法最重要的是确定初始单元及其内外部信息，此处从边界面入手，遍历边界面，在当前三角化内查找包含当前判断边界面的网格单元，若只有一个网格单元，则该网格单元必是内部单元
    std::vector<int> elemNum_include_face;
    _MESH_PROCESS mesh_process;
    for (std::vector<FACE>::iterator iter = su_mesh->boundary_face.begin(); iter != su_mesh->boundary_face.end(); ++iter)
    {
        std::vector<int>().swap(elemNum_include_face);
        mesh_process.FindAwl(su_mesh, *iter, &elemNum_include_face, "fast");
        if (elemNum_include_face.size() == 1)
        {
            ini_elemNum = elemNum_include_face.front();
            break;
        }
    }
    external_elem_judge[ini_elemNum] = 1;
    std::deque<int> elemNum_wait;
    std::vector<int> elemNum_succ;
    elemNum_wait.push_back(ini_elemNum);
    elemNum_succ.push_back(ini_elemNum);
    ELEM elem_tp;
    FACE face_tp;
    int elemNum_tp;
    int elemNum_tp_neig;
    while (elemNum_succ.size() < su_mesh->elem_num)
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
    }
    for (int i = 0; i < su_mesh->elem_num; i++)
    {
        if (external_elem_judge[i] == -1)
            elemNum_External.push_back(i);
        if (external_elem_judge[i] == 0)
            std::cout << i << ' ';
    }
    free(external_elem_judge);
    return elemNum_External;
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

void _BOUNDARY_RECOVERY::Removal_NodeElem(_SU_MESH *su_mesh, std::vector<int> elemNum_Remove, int nodeNum_Remove[8], int type)
{
    int cnt_node = 0; // 记录待删除节点的个数
    if (type == 1)
    {
        // 为简化删除节点流程，当查找到待删除节点时，使用容器内最后一个有效节点来替换此位置，在程序结束时，删除最后cnt_node个节点，最大限度地保证了整个网格的相邻信息不变
        // 为避免不必要的麻烦，将nodeNum_Remove降序排序，从nodeNum_Remove的最后一个节点编号开始检索替换
        std::sort(nodeNum_Remove, nodeNum_Remove + 8, std::greater<>());
        // 将nodeNum_Remove内所有节点放到node容器最后
        for (int i = 0; i < 8; i++)
        {
            // 将该节点与node容器的最后一个有效节点交换位置
            // 如果该节点就位于node容器的最后一个有效节点位置，则不交换
            if (nodeNum_Remove[i] == su_mesh->node_num - 1 - cnt_node)
            {
                cnt_node++;
                continue;
            }
            else
                ReplaceNode_two(su_mesh, nodeNum_Remove[i], su_mesh->node_num - 1 - cnt_node++);
        }
    }
    int cnt_elem = 0; // 记录待删除网格单元的个数
    // 将elemNum_Remove内所有网格单元放到elem容器最后
    for (int i = int(elemNum_Remove.size()) - 1; i >= 0; i--)
    {
        // 将该网格单元与elem容器的最后一个有效网格单元交换位置
        // 如果该网格单元就位于elem容器的最后一个有效网格单元位置，则不交换
        if (elemNum_Remove.at(i) == su_mesh->elem.size() - 1 - cnt_elem)
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
    if (type == 1)
    {
        for (int i = 0; i < cnt_node; i++)
            Removal_LastNode(su_mesh);
    }
    return;
}

void _BOUNDARY_RECOVERY::Removal_ExGrid(_SU_MESH *su_mesh, int type) // 去掉外部单元并缩减容器
{
    int nodeNum_Remove[8];
    std::vector<int> elemNum_Remove; // 声明一个容器，用来储存待移除网格单元编号
    if (type == 1)
    {
        std::copy(su_mesh->Delaunay_Frame_numPos, su_mesh->Delaunay_Frame_numPos + 8, nodeNum_Remove); // 声明数组，用来指向储存待移除节点的连续内存空间
        for (int i = 0; i < su_mesh->elem_num; i++)
            if (su_mesh->elem.at(i).form[3] >= su_mesh->InitNode_num)
                elemNum_Remove.push_back(i);
    }
    if (type == 2)
        elemNum_Remove = External_Elem_Lookup(su_mesh);
    Removal_NodeElem(su_mesh, elemNum_Remove, nodeNum_Remove, type); // 在node容器内删除节点与单元并更新所有信息
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
                exit(-1);
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
                exit(-1);
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

bool _BOUNDARY_RECOVERY::Decompose_Pathl(_SU_MESH *su_mesh, Pathl pathl)
{
    // 通过路径元type域值，判断当前路径元类型，对不同类型进行不同形式的分解
    // 单边型（包含点边型）
    if ((pathl.type[0] == 1 && pathl.type[1] == 2) || (pathl.type[0] == 2 && pathl.type[1] == 0))
    {
        // 单边型会分解成两个网格单元
        pathl.Decom_elem_num = 2;
        pathl.Decom_elem = (ELEM *)malloc(sizeof(ELEM) * pathl.Decom_elem_num);
    }
    return true;
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
                        exit(-1);
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
            exit(-1);
        }
    }
    // 如果路径中有四个路径元
    else if (path.size() == 4)
    {
    }
    else
    {
        std::cout << path.size() << ' ';
        //std::cout << "The size of the existing path is greater than 2!\n";
    }
    return;
}

std::vector<int> _BOUNDARY_RECOVERY::FindSet(_SU_MESH *su_mesh, FACE face_recovery)
{
    std::vector<int> s;
    return s;
}

void _BOUNDARY_RECOVERY::Recovery_Boundary_face(_SU_MESH *su_mesh, FACE face_recovery)
{
    return;
}

void _BOUNDARY_RECOVERY::Recovery_Boundary(_SU_MESH *su_mesh)
{
    // 首先恢复边界边，查找所有不在当前三角化内的边界边
    std::vector<EDGE> edge_wait_recovery; // 储存待恢复的边界边
    std::vector<int> elemNum_IncludeEdge;
    _MESH_PROCESS mesh_process;
    for (std::vector<EDGE>::iterator iter = su_mesh->boundary_edge.begin(); iter != su_mesh->boundary_edge.end(); ++iter)
    {
        std::vector<int>().swap(elemNum_IncludeEdge);
        mesh_process.FindRing(su_mesh, *iter, &elemNum_IncludeEdge, "fast");
        if (elemNum_IncludeEdge.empty())
            edge_wait_recovery.push_back(*iter);
    }
    // 一个个恢复边界边
    for (std::vector<EDGE>::iterator iter = edge_wait_recovery.begin(); iter != edge_wait_recovery.end(); ++iter)
        Recovery_Boundary_edge(su_mesh, *iter);
    // 再恢复边界面
    //std::vector<FACE> face_wait_recovery; // 储存待恢复的边界面
    //std::vector<int> elemNum_IncludeFace;
    //for (std::vector<FACE>::iterator iter = su_mesh->boundary_face.begin(); iter != su_mesh->boundary_face.end(); ++iter)
    //{
    //	std::vector<int>().swap(elemNum_IncludeFace);
    //	mesh_process.FindAwl(su_mesh, *iter, &elemNum_IncludeFace, "fast");
    //	if (elemNum_IncludeFace.empty())
    //		face_wait_recovery.push_back(*iter);
    //}
    //if (!face_wait_recovery.empty())
    //{
    //	std::cout << "There are boundary surfaces to restore!\n";
    //	exit(-1);
    //}
    // 一个个恢复边界面
    //for (std::vector<FACE>::iterator iter = face_wait_recovery.begin(); iter != face_wait_recovery.end(); ++iter)
    //	Recovery_Boundary_face(su_mesh, *iter);
    return;
}

bool _BOUNDARY_RECOVERY::operator==(const _BOUNDARY_RECOVERY &other) const
{
    return false;
}
