#include "Su_Mesh.h"

void _BOUNDARY_POINT::IniDelaunay(_SU_MESH *su_mesh, int aaa)
{
    // 储存并查找边界边长度信息
    NODE node_tp;
    FACE face_tp;
    EDGE edge_tp;
    std::vector<FACE>::iterator face_iter;
    double value_tp;
    _DATA_PROCESS data_process;
    // 查找边界边
    for (face_iter = su_mesh->boundary_face.begin(); face_iter != su_mesh->boundary_face.end(); ++face_iter)
    {
        for (int i = 0; i < 2; i++)
            for (int j = i + 1; j < DIM; j++)
            {
                value_tp = data_process.get_dist(su_mesh->node.at(face_iter->form[i]).pos, su_mesh->node.at(face_iter->form[j]).pos);
                if (su_mesh->longest_border_edge < value_tp)
                    su_mesh->longest_border_edge = value_tp;
                if (su_mesh->shortest_border_edge > value_tp)
                    su_mesh->shortest_border_edge = value_tp;
                edge_tp.form[0] = face_iter->form[i];
                edge_tp.form[1] = face_iter->form[j];
                if (std::find(su_mesh->boundary_edge.begin(), su_mesh->boundary_edge.end(), edge_tp) == su_mesh->boundary_edge.end())
                    su_mesh->boundary_edge.push_back(edge_tp);
            }
    }
    // 依据每个每个节点组成边长度的均值给节点赋值密度信息
    int cnt = 0;
    double length = 0;
    std::vector<int> edgeNum;
    std::vector<int>::iterator edgeNum_iter;
    std::vector<EDGE>::iterator edge_iter;
    for (int nodeNum_iter = 0; nodeNum_iter < su_mesh->node_num; nodeNum_iter++)
    {
        cnt = 0;
        length = 0;
        std::vector<int>().swap(edgeNum);
        for (edge_iter = su_mesh->boundary_edge.begin(); edge_iter != su_mesh->boundary_edge.end(); ++edge_iter)
        {
            if (edge_iter->form[0] == nodeNum_iter || edge_iter->form[1] == nodeNum_iter)
                edgeNum.push_back(cnt);
            cnt++;
        }
        length = su_mesh->longest_border_edge;
        for (edgeNum_iter = edgeNum.begin(); edgeNum_iter != edgeNum.end(); ++edgeNum_iter)
        {
            //length += data_process.get_dist(su_mesh->node.at(su_mesh->boundary_edge.at(*edgeNum_iter).form[0]).pos, su_mesh->node.at(su_mesh->boundary_edge.at(*edgeNum_iter).form[1]).pos);
            value_tp = data_process.get_dist(su_mesh->node.at(su_mesh->boundary_edge.at(*edgeNum_iter).form[0]).pos, su_mesh->node.at(su_mesh->boundary_edge.at(*edgeNum_iter).form[1]).pos);
            length = length > value_tp ? value_tp : length;
        }
        //length /= double(edgeNum.size());
        if (length < su_mesh->shortest_border_edge * 2.0)
            length *= 1.5;
        su_mesh->node.at(nodeNum_iter).spac = length;
    }
    if (aaa == 1)
    {
        // 将非常短的边连接起来
        value_tp = su_mesh->shortest_border_edge * 50.0;
        //su_mesh->shortest_border_edge *= 500.0;
        for (std::vector<EDGE>::iterator edge_iter = su_mesh->boundary_edge.begin(); edge_iter != su_mesh->boundary_edge.end();)
        {
            if (data_process.get_dist(su_mesh->node.at(edge_iter->form[0]).pos, su_mesh->node.at(edge_iter->form[1]).pos) < value_tp)
            {
                node_tp = (su_mesh->node.at(edge_iter->form[0]) + su_mesh->node.at(edge_iter->form[1])) * 0.5;
                su_mesh->node.at(edge_iter->form[0]) = node_tp;
                if (edge_iter->form[1] != su_mesh->node_num - 1)
                    std::swap(su_mesh->node.at(edge_iter->form[1]), su_mesh->node.at(long long(su_mesh->node_num) - 1));
                su_mesh->node.pop_back();
                for (std::vector<EDGE>::iterator iter = su_mesh->boundary_edge.begin(); iter != su_mesh->boundary_edge.end(); ++iter)
                {
                    if (iter->form[0] == edge_iter->form[1])
                        iter->form[0] = edge_iter->form[0];
                    else if (iter->form[1] == edge_iter->form[1])
                        iter->form[1] = edge_iter->form[0];
                    if (iter->form[0] == su_mesh->node_num - 1)
                        iter->form[0] = edge_iter->form[1];
                    else if (iter->form[1] == su_mesh->node_num - 1)
                        iter->form[1] = edge_iter->form[1];
                }
                for (face_iter = su_mesh->boundary_face.begin(); face_iter != su_mesh->boundary_face.end(); ++face_iter)
                {
                }
                su_mesh->node_num--;
                edge_iter = su_mesh->boundary_edge.erase(edge_iter);
                continue;
            }
            ++edge_iter;
        }
        su_mesh->InitNode_num = su_mesh->node_num;
    }
    su_mesh->longest_distance = su_mesh->longest_border_edge * 1.3;
    //double density = su_mesh->longest_border_edge * 0.4 + su_mesh->shortest_border_edge * 0.4;
    //for (std::vector<NODE>::iterator node_iter = su_mesh->node.begin(); node_iter != su_mesh->node.end(); node_iter++)
    //	node_iter->spac = density;
    // 定义一个数组，储存输入六面体网格的初始边框坐标x、y、z方向上下限，分别为：x_min,x_max,y_min,y,max,z.min,z.max
    double Init_boundary_pos[] = {0, 0, 0, 0, 0, 0};
    // 遍历node容器
    for (std::vector<NODE>::iterator node_iter = su_mesh->node.begin(); node_iter != su_mesh->node.end(); ++node_iter)
    {
        if (Init_boundary_pos[0] > node_iter->pos[0])
            Init_boundary_pos[0] = node_iter->pos[0];
        if (Init_boundary_pos[1] < node_iter->pos[0])
            Init_boundary_pos[1] = node_iter->pos[0];
        if (Init_boundary_pos[2] > node_iter->pos[1])
            Init_boundary_pos[2] = node_iter->pos[1];
        if (Init_boundary_pos[3] < node_iter->pos[1])
            Init_boundary_pos[3] = node_iter->pos[1];
        if (Init_boundary_pos[4] > node_iter->pos[2])
            Init_boundary_pos[4] = node_iter->pos[2];
        if (Init_boundary_pos[5] < node_iter->pos[2])
            Init_boundary_pos[5] = node_iter->pos[2];
    }
    double Init_DeDelaunay_Frame[] = {-1, -1, -1, -1, -1, -1}; // 定义初始Delaunay三角化四边形边框，分别为：x_min,x_max,y_min,y,max,z.min,z.max
    // 如果输入图形初始方形边框是正方体，则用各自边来赋值
    if (((Init_boundary_pos[1] - Init_boundary_pos[0]) == (Init_boundary_pos[3] - Init_boundary_pos[2])) &&
        ((Init_boundary_pos[3] - Init_boundary_pos[2]) == (Init_boundary_pos[5] - Init_boundary_pos[4])))
    {
        Init_DeDelaunay_Frame[0] = Init_boundary_pos[0] - (Init_boundary_pos[1] - Init_boundary_pos[0]) / 2;
        Init_DeDelaunay_Frame[1] = Init_boundary_pos[1] + (Init_boundary_pos[1] - Init_boundary_pos[0]) / 2;
        Init_DeDelaunay_Frame[2] = Init_boundary_pos[2] - (Init_boundary_pos[3] - Init_boundary_pos[2]) / 2;
        Init_DeDelaunay_Frame[3] = Init_boundary_pos[3] + (Init_boundary_pos[3] - Init_boundary_pos[2]) / 2;
        Init_DeDelaunay_Frame[4] = Init_boundary_pos[4] - (Init_boundary_pos[5] - Init_boundary_pos[4]) / 2;
        Init_DeDelaunay_Frame[5] = Init_boundary_pos[5] + (Init_boundary_pos[5] - Init_boundary_pos[4]) / 2;
    }
    // 如果输入图形初始方形边框不是正方体，是长方体，则用最长的边来赋值
    else
    {
        double tp = std::max(std::max((Init_boundary_pos[1] - Init_boundary_pos[0]), (Init_boundary_pos[3] - Init_boundary_pos[2])),
                             (Init_boundary_pos[5] - Init_boundary_pos[4]));
        Init_DeDelaunay_Frame[0] = Init_boundary_pos[0] - tp / 2;
        Init_DeDelaunay_Frame[1] = Init_boundary_pos[1] + tp / 2;
        Init_DeDelaunay_Frame[2] = Init_boundary_pos[2] - tp / 2;
        Init_DeDelaunay_Frame[3] = Init_boundary_pos[3] + tp / 2;
        Init_DeDelaunay_Frame[4] = Init_boundary_pos[4] - tp / 2;
        Init_DeDelaunay_Frame[5] = Init_boundary_pos[5] + tp / 2;
    }
    // 正方体边框有八个顶点
    // 第一个顶点
    node_tp.pos[0] = Init_DeDelaunay_Frame[0];
    node_tp.pos[1] = Init_DeDelaunay_Frame[2];
    node_tp.pos[2] = Init_DeDelaunay_Frame[4];
    su_mesh->node.push_back(node_tp);
    su_mesh->Delaunay_Frame_numPos[0] = (su_mesh->node_num)++;
    // 第二个顶点
    node_tp.pos[0] = Init_DeDelaunay_Frame[1];
    node_tp.pos[1] = Init_DeDelaunay_Frame[2];
    node_tp.pos[2] = Init_DeDelaunay_Frame[4];
    su_mesh->node.push_back(node_tp);
    su_mesh->Delaunay_Frame_numPos[1] = (su_mesh->node_num)++;
    // 第三个顶点
    node_tp.pos[0] = Init_DeDelaunay_Frame[1];
    node_tp.pos[1] = Init_DeDelaunay_Frame[3];
    node_tp.pos[2] = Init_DeDelaunay_Frame[4];
    su_mesh->node.push_back(node_tp);
    su_mesh->Delaunay_Frame_numPos[2] = (su_mesh->node_num)++;
    // 第四个顶点
    node_tp.pos[0] = Init_DeDelaunay_Frame[0];
    node_tp.pos[1] = Init_DeDelaunay_Frame[3];
    node_tp.pos[2] = Init_DeDelaunay_Frame[4];
    su_mesh->node.push_back(node_tp);
    su_mesh->Delaunay_Frame_numPos[3] = (su_mesh->node_num)++;
    // 第五个顶点
    node_tp.pos[0] = Init_DeDelaunay_Frame[0];
    node_tp.pos[1] = Init_DeDelaunay_Frame[2];
    node_tp.pos[2] = Init_DeDelaunay_Frame[5];
    su_mesh->node.push_back(node_tp);
    su_mesh->Delaunay_Frame_numPos[4] = (su_mesh->node_num)++;
    // 第六个顶点
    node_tp.pos[0] = Init_DeDelaunay_Frame[1];
    node_tp.pos[1] = Init_DeDelaunay_Frame[2];
    node_tp.pos[2] = Init_DeDelaunay_Frame[5];
    su_mesh->node.push_back(node_tp);
    su_mesh->Delaunay_Frame_numPos[5] = (su_mesh->node_num)++;
    // 第七个顶点
    node_tp.pos[0] = Init_DeDelaunay_Frame[1];
    node_tp.pos[1] = Init_DeDelaunay_Frame[3];
    node_tp.pos[2] = Init_DeDelaunay_Frame[5];
    su_mesh->node.push_back(node_tp);
    su_mesh->Delaunay_Frame_numPos[6] = (su_mesh->node_num)++;
    // 第八个顶点
    node_tp.pos[0] = Init_DeDelaunay_Frame[0];
    node_tp.pos[1] = Init_DeDelaunay_Frame[3];
    node_tp.pos[2] = Init_DeDelaunay_Frame[5];
    su_mesh->node.push_back(node_tp);
    su_mesh->Delaunay_Frame_numPos[7] = (su_mesh->node_num)++;
    // 用该八个顶点生成初始初始Delaunay三角化，包含5个四面体网格单元
    ELEM elem_tp;
    // 第一个四面体网格单元
    int value_1[] = {0 + su_mesh->InitNode_num, 1 + su_mesh->InitNode_num, 2 + su_mesh->InitNode_num, 5 + su_mesh->InitNode_num, -1, 3, -1, -1};
    elem_tp = value_1;
    su_mesh->elem.push_back(elem_tp);
    su_mesh->elem_num += 1;
    // 第二个四面体网格单元
    int value_2[] = {0 + su_mesh->InitNode_num, 2 + su_mesh->InitNode_num, 3 + su_mesh->InitNode_num, 7 + su_mesh->InitNode_num, -1, -1, 3, -1};
    elem_tp = value_2;
    su_mesh->elem.push_back(elem_tp);
    su_mesh->elem_num += 1;
    // 第三个四面体网格单元
    int value_3[] = {2 + su_mesh->InitNode_num, 5 + su_mesh->InitNode_num, 6 + su_mesh->InitNode_num, 7 + su_mesh->InitNode_num, -1, -1, 3, -1};
    elem_tp = value_3;
    su_mesh->elem.push_back(elem_tp);
    su_mesh->elem_num += 1;
    // 第四个四面体网格单元
    int value_4[] = {0 + su_mesh->InitNode_num, 2 + su_mesh->InitNode_num, 5 + su_mesh->InitNode_num, 7 + su_mesh->InitNode_num, 2, 4, 1, 0};
    elem_tp = value_4;
    su_mesh->elem.push_back(elem_tp);
    su_mesh->elem_num += 1;
    // 第五个四面体网格单元
    int value_5[] = {0 + su_mesh->InitNode_num, 4 + su_mesh->InitNode_num, 5 + su_mesh->InitNode_num, 7 + su_mesh->InitNode_num, -1, 3, -1, -1};
    elem_tp = value_5;
    su_mesh->elem.push_back(elem_tp);
    su_mesh->elem_num += 1;
    return;
}

void _BOUNDARY_POINT::Rounding(_SU_MESH *su_mesh)
{
    for (std::vector<NODE>::iterator iter = su_mesh->node.begin(); iter != su_mesh->node.end(); ++iter)
        for (int i = 0; i < DIM; i++)
            iter->pos[i] = ceil(iter->pos[i]);
    return;
}

void _BOUNDARY_POINT::Insert_BoundaryPoint(_SU_MESH *su_mesh)
{
    _INSERT_POINT insert_Point;
    int nodeNum_Last_succ = -1; // 储存上一次成功插入的边界点编号，初始化为-1
    int cnt;
    for (cnt = 0; cnt < su_mesh->InitNode_num; cnt++)
    {
        // 插入边界点，边界点插入时，插入点程序不会返回3
        if (insert_Point.Insert_point(su_mesh, nodeNum_Last_succ, cnt, su_mesh->node.at(cnt), -1) == 2)
            su_mesh->nodeNum_degradation.push_back(cnt);
        else
            nodeNum_Last_succ = cnt;
    }
    // 插完一遍边界点后，遍历su_mesh->nodeNum_degradation内节点，重新插入他们
    bool judgment; // 存储上次while循环是否成功插入节点
    while (!su_mesh->nodeNum_degradation.empty())
    {
        judgment = false;
        nodeNum_Last_succ = su_mesh->nodeNum_degradation.front() - 1;
        for (std::vector<int>::iterator iter = su_mesh->nodeNum_degradation.begin(); iter != su_mesh->nodeNum_degradation.end();)
        {
            if (insert_Point.Insert_point(su_mesh, nodeNum_Last_succ, *iter, su_mesh->node.at(*iter), -1) == 2)
                ++iter;
            else
            {
                nodeNum_Last_succ = *iter;
                iter = su_mesh->nodeNum_degradation.erase(iter);
                judgment = true;
            }
        }
        if (!judgment)
            break;
    }
    NODE node_before;
    double disturbance_delta = (su_mesh->shortest_border_edge * 0.7 + su_mesh->longest_border_edge * 0.3) / 100000; // 储存点扰动距离，定义为最短边界边长度的1/100000
    NODE node_delta = {disturbance_delta, disturbance_delta, disturbance_delta};
    // 如果还存在不能插入的边界点，就需要对这些边界点做扰动了，扰动后再进行插入操作
    if (!su_mesh->nodeNum_degradation.empty())
    {
        nodeNum_Last_succ = su_mesh->nodeNum_degradation.front() - 1;
        for (std::vector<int>::iterator iter = su_mesh->nodeNum_degradation.begin(); iter != su_mesh->nodeNum_degradation.end(); ++iter)
        {
            node_before = su_mesh->node.at(*iter); // 储存点扰动前坐标
            su_mesh->node.at(*iter) = su_mesh->node.at(*iter) + node_delta;
            cnt = 0;
            while (insert_Point.Insert_point(su_mesh, *iter - 1, *iter, su_mesh->node.at(*iter), -1) == 2)
            {
                cnt++;
                if (cnt > 20)
                {
                    std::cout << "drop-dead halt!\n";
                    exit(-1);
                }
                su_mesh->node.at(*iter) = su_mesh->node.at(*iter) + node_delta;
            }
            node_before.elem = su_mesh->node.at(*iter).elem;
            //su_mesh->node.at(*iter) = node_before;
            nodeNum_Last_succ = *iter;
        }
        // std::cout << "Boundary points exist and cannot be inserted!\n";
        // exit(-1);
    }
    return;
}
