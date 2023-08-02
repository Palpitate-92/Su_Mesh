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
                    std::cout << "Decompose_Pathl run error, the guess is that the path lookup failed!\n", exit(-1);
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
                // 得到分解后的三个网格单元
                if (pathl->Decom_elem)
                {
                    if (pathl->Decom_type_two_sides = 'S')
                    {
                        *pathl->Decom_elem = ELEM(ExplodeSameNode_num, ExplodeOppoNode_num, STEINER_NOD, STEINER_NOD, -1, -1, -1, -1);
                        *(pathl->Decom_elem + 1) = ELEM(OppoEdge.form[1], ExplodeOppoNode_num, STEINER_NOD, STEINER_NOD, -1, -1, -1, -1);
                        *(pathl->Decom_elem + 2) = ELEM(OppoEdge.form[0], OppoEdge.form[1], ExplodeOppoNode_num, STEINER_NOD, -1, -1, -1, pathl->neig[mesh_process.Elem_Include_Node(*pathl, ExplodeSameNode_num)]);
                    }
                    else
                    {
                        *pathl->Decom_elem = ELEM(ExplodeSameNode_num, ExplodeOppoNode_num, STEINER_NOD, STEINER_NOD, -1, -1, -1, -1);
                        /*
                        *
                        *
                        *
                         */
                        *(pathl->Decom_elem + 1) = ELEM(OppoEdge.form[1], ExplodeOppoNode_num, STEINER_NOD, STEINER_NOD, -1, -1, -1, -1);
                        *(pathl->Decom_elem + 2) = ELEM(OppoEdge.form[0], OppoEdge.form[1], ExplodeOppoNode_num, STEINER_NOD, -1, -1, -1, pathl->neig[mesh_process.Elem_Include_Node(*pathl, ExplodeSameNode_num)]);
                    }
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
            // 得到该相邻网格边的相对网格边
            EDGE ExplodeAdjacentOppoEdge = mesh_process.Edge_Opposite_Edge(*pathl, ExplodeAdjacentEdge);
            // 得到分解后的五个网格单元
            if (pathl->Decom_elem)
            {
                *pathl->Decom_elem = ELEM(ExplodeFace_1.form[0], ExplodeFace_1.form[1], STEINER_NOD, STEINER_NOD, -1, -1, -1, -1);
                *(pathl->Decom_elem + 1) = ELEM(ExplodeFace_1.form[0], ExplodeFace_1.form[2], STEINER_NOD, STEINER_NOD, -1, -1, -1, -1);
                *(pathl->Decom_elem + 2) = ELEM(ExplodeFace_1.form[1], ExplodeFace_1.form[2], STEINER_NOD, STEINER_NOD, -1, -1, -1, -1);
                *(pathl->Decom_elem + 3) = ELEM(ExplodeAdjacentEdge.form[0], ExplodeAdjacentOppoEdge.form[0], ExplodeAdjacentOppoEdge.form[1], STEINER_NOD, -1, -1, -1, pathl->neig[mesh_process.Elem_Include_Node(*pathl, ExplodeAdjacentEdge.form[1])]);
                *(pathl->Decom_elem + 4) = ELEM(ExplodeAdjacentEdge.form[1], ExplodeAdjacentOppoEdge.form[0], ExplodeAdjacentOppoEdge.form[1], STEINER_NOD, -1, -1, -1, pathl->neig[mesh_process.Elem_Include_Node(*pathl, ExplodeAdjacentEdge.form[0])]);
            }
        }
        else
        {
            std::cout << "Pathl decompose error!\n";
            exit(-1);
        }
    }
    return;
}

void _BOUNDARY_RECOVERY::Pathl_Generate_GridCell(_SU_MESH *su_mesh, Pathl pathl, std::vector<int> *elemNum_adjacent, std::vector<FACE> *face_adjacent)
{
    // 声明两个迭代器
    int elemNum_iter = -1;
    std::vector<FACE>::iterator face_iter;
    _MESH_PROCESS mesh_process;
    FACE *face_judge = NULL;
    int *elemNum_judge = NULL;
    int faceNum_cnt = 0;
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
                steiner_node_num = su_mesh->node_num++;
            }
        }
        else
        {
            if (su_mesh->node.back() == NODE(*pathl.pot))
                steiner_node_num = su_mesh->node_num - 1;
            else
            {
                su_mesh->node.push_back(NODE(*pathl.pot));
                steiner_node_num = su_mesh->node_num++;
            }
        }
        // 根据steiner点编号，更新路径元分解生成的网格单元的节点信息
        (pathl.Decom_elem + 0)->form[3] = steiner_node_num;
        (pathl.Decom_elem + 1)->form[3] = steiner_node_num;
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
    }
    // 对边型和邻边型的两个相交图形都是边
    else if (pathl.type[0] == 2 && pathl.type[1] == 2)
    {
        // 首先将steiner点压入node容器，并储存该steiner点在网格中的节点编号
        // 先判断当前待压入的steiner点是否已被压入
        int *steiner_node_num = new int[2];
        for (int i = 0; i < 2; i++)
        {
            if (*(su_mesh->node.end() - 0) == NODE(*(pathl.pot + i)))
                *(steiner_node_num + i) = su_mesh->node_num - 1;
            else if (*(su_mesh->node.end() - 1) == NODE(*(pathl.pot + i)))
                *(steiner_node_num + i) = su_mesh->node_num - 2;
            else
            {
                su_mesh->node.push_back(NODE(*(pathl.pot + i)));
                *(steiner_node_num + i) = su_mesh->node_num++;
            }
        }
        // 根据steiner点编号，更新路径元分解生成的网格单元的节点信息，然后更新相邻信息，插入elem容器，最后储存待判断网格面
        // 对边型，“D”型
        if (pathl.Decom_type_two_sides = 'D')
        {
            // 首先更新节点信息
            for (int i = 0; i < 4; i++)
                (pathl.Decom_elem + i)->form[2] = *(steiner_node_num + 0), (pathl.Decom_elem + i)->form[3] = *(steiner_node_num + 1);
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
        }
        // 邻边“S”型
        else if (pathl.Decom_type_two_sides = 'S')
        {
            // 首先更新节点信息
            for (int i = 0; i < 2; i++)
                (pathl.Decom_elem + i)->form[2] = *(steiner_node_num + 0), (pathl.Decom_elem + i)->form[3] = *(steiner_node_num + 1);
            (pathl.Decom_elem + 2)->form[3] = *(steiner_node_num + 0);
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
        }
        // 邻边“Z”型
        else if (pathl.Decom_type_two_sides = 'Z')
        {
            // 首先更新节点信息
            for (int i = 0; i < 2; i++)
                (pathl.Decom_elem + i)->form[2] = *(steiner_node_num + 0), (pathl.Decom_elem + i)->form[3] = *(steiner_node_num + 1);
            (pathl.Decom_elem + 2)->form[3] = *(steiner_node_num + 0);
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
        }
        else
            std::cout << "Bilateral pathl's Decom_type_two_sides set error!\n", exit(-1);
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
                steiner_node_num = su_mesh->node_num++;
            }
        }
        else
        {
            if (su_mesh->node.back() == NODE(*pathl.pot))
                steiner_node_num = su_mesh->node_num - 1;
            else
            {
                su_mesh->node.push_back(NODE(*pathl.pot));
                steiner_node_num = su_mesh->node_num++;
            }
        }
        // 根据steiner点编号，更新路径元分解生成的网格单元的节点信息
        for (int i = 0; i < 3; i++)
            (pathl.Decom_elem + i)->form[3] = steiner_node_num;
        // 先更新所有能在该步骤下进行更新的相邻信息，再将新生成的两个网格单元插入elem容器
        // 得到新生成的网格单元使用的网格单元编号
        int pathl_elem_num[] = {pathl.elem_num, su_mesh->elem_num, su_mesh->elem_num + 1};
        // 更新这三个网格单元的相邻信息
        (pathl.Decom_elem + 0)->neig[1] = pathl_elem_num[1], (pathl.Decom_elem + 0)->neig[2] = pathl_elem_num[2];
        (pathl.Decom_elem + 1)->neig[1] = pathl_elem_num[1], (pathl.Decom_elem + 1)->neig[2] = pathl_elem_num[0];
        su_mesh->elem.at((pathl.Decom_elem + 1)->neig[3]).neig[mesh_process.AdjacentElem_pos(su_mesh->elem.at((pathl.Decom_elem + 1)->neig[3]), pathl.elem_num)] = pathl_elem_num[1];
        (pathl.Decom_elem + 2)->neig[1] = pathl_elem_num[2], (pathl.Decom_elem + 2)->neig[2] = pathl_elem_num[0];
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
    }
    // 边面型（面边型）
    else if ((pathl.type[0] == 2 && pathl.type[1] == 3) || (pathl.type[0] == 3 && pathl.type[1] == 2))
    {
        // 边面型、面边型的网格生成步骤相同，但其steiner点的储存位置不同
        // 首先将steiner点压入node容器，并储存该steiner点在网格中的节点编号
        // 先判断当前待压入的steiner点是否已被压入
        int *steiner_node_num = new int[2];
        for (int i = 0; i < 2; i++)
        {
            if (*(su_mesh->node.end() - 0) == NODE(*(pathl.pot + i)))
                *(steiner_node_num + i) = su_mesh->node_num - 1;
            else if (*(su_mesh->node.end() - 1) == NODE(*(pathl.pot + i)))
                *(steiner_node_num + i) = su_mesh->node_num - 2;
            else
            {
                su_mesh->node.push_back(NODE(*(pathl.pot + i)));
                *(steiner_node_num + i) = su_mesh->node_num++;
            }
        }
        // 根据steiner点编号，更新路径元分解生成的网格单元的节点信息
        for (int i = 0; i < 3; i++)
            (pathl.Decom_elem + i)->form[2] = *(steiner_node_num + 0), (pathl.Decom_elem + i)->form[3] = *(steiner_node_num + 1);
        if (pathl.type[0] == 2)
            (pathl.Decom_elem + 3)->form[3] = *(steiner_node_num + 0);
        else
            (pathl.Decom_elem + 3)->form[3] = *(steiner_node_num + 1);
        // 先更新所有能在该步骤下进行更新的相邻信息，再将新生成的两个网格单元插入elem容器
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

        //su_mesh->elem.at(pathl.Decom_elem->neig[3]).neig[mesh_process.AdjacentElem_pos(su_mesh->elem.at(pathl.Decom_elem->neig[3]), pathl.elem_num)] = pathl.elem_num;
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
        delete[] steiner_node_num;
    }
    // 双面型
    else if (pathl.type[0] == 3 && pathl.type[1] == 3)
    {
    }
    else
        std::cout << "Pathl generate grid cell error!\n", exit(-1);
    // 更新路径元与路径元之间的相邻信息
    // 在face_adjacent容器内查找这些网格面，若不存在则直接压入容器并将这个网格面代表的网格单元编号一并压入容器elemNum_adjacent，若存在则取出相对于的网格面与网格单元编号，并进行相邻信息更新
    FACE face_tp;
    int elemNum_tp = -1;
    // 保证数据有效
    if (face_judge == NULL || elemNum_judge == NULL)
        std::cout << "Pathl generate grid cell error!\n", exit(-1);
    else
        for (int i = 0; i < faceNum_cnt; i++)
        {
            face_judge[i].Sort();
            if ((face_iter = std::find(face_adjacent->begin(), face_adjacent->end(), face_judge[i])) == face_adjacent->end())
            {
                face_adjacent->push_back(face_judge[i]);
                elemNum_adjacent->push_back(elemNum_judge[i]);
            }
            else
            {
                elemNum_iter = std::distance(face_adjacent->begin(), face_iter);
                if (elemNum_iter < int(elemNum_adjacent->size()))
                    elemNum_tp = elemNum_adjacent->at(elemNum_iter);
                else
                    std::cout << "Pathl generate grid cell error, the elemNum_iter is out of bounds!\n", exit(-1);
                // 从容器内删除这两个值
                face_adjacent->erase(face_iter);
                elemNum_adjacent->erase(elemNum_adjacent->begin() + elemNum_iter);
                // 修改包含该网格面的两个网格单元的相邻信息
                su_mesh->elem.at(elemNum_judge[i]).neig[mesh_process.Face_Opposite_Node(su_mesh->elem.at(elemNum_judge[i]), face_judge[i])] = elemNum_tp;
                su_mesh->elem.at(elemNum_tp).neig[mesh_process.Face_Opposite_Node(su_mesh->elem.at(elemNum_tp), face_judge[i])] = elemNum_judge[i];
            }
        }
    delete[] face_judge;
    face_judge = NULL;
    delete[] elemNum_judge;
    elemNum_judge = NULL;
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
    //else if (path.size() == 4)
    //{
    //}
    else
    {
        // 遍历每个路径元，分别进行分解和生成网格单元，同时确保相邻信息的准确性
        // 声明两个容器，分别存储待更新相邻信息的网格面与相对应的网格单元编号
        std::vector<int> elemNum_adjacent;
        std::vector<FACE> face_adjacent;
        // 先对路径元进行分解操作
        Decompose_Pathl(&path);
        //elem_new[0].neig[1] = elemNum_adjacent.at(elemNum_iter = std::distance(face_adjacent.begin(), face_iter = std::find(face_adjacent.begin(), face_adjacent.end(), face_tp[1])));
        for (std::vector<Pathl>::iterator iter = path.begin(); iter != path.end(); ++iter)
        {
            // 再依据路径元当前类型，将分解后的网格压入elem容器，形成路径元的完整分解生成过程
            Pathl_Generate_GridCell(su_mesh, *iter, &elemNum_adjacent, &face_adjacent);
        }
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
