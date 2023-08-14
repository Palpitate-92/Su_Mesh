#include "Su_Mesh.h"

bool _CAVITY::Find_Cavity(_SU_MESH *su_mesh, std::vector<int> *elemNum_Cavity, NODE node_Insert, int elemNum_IncludeNodeIns)
{
    _DATA_PROCESS data_process;
    std::vector<int> elemNum_wait;                  // 创建一个容器，用来储存待判断的单元
    std::vector<int> elemNum_succ;                  // 创建一个容器，用来储存判断过的单元
    elemNum_succ.push_back(elemNum_IncludeNodeIns); // 直接压入待插入节点所处的单元编号到elemNum_succ，不用判断
    for (int i = 0; i < DIM + 1; i++)               // 首先压入待插入节点所处的单元的相邻单元到elemNum_wait
        if (su_mesh->elem.at(elemNum_IncludeNodeIns).neig[i] != -1)
            elemNum_wait.push_back(su_mesh->elem.at(elemNum_IncludeNodeIns).neig[i]);
    int elemNum_tp;
    ELEM elem_tp;
    while (!elemNum_wait.empty()) // 只要elemNum_wait内有元素就继续查找
    {
        elemNum_tp = elemNum_wait.front(); // 取出elemNum_wait中的第一个元素并擦除
        elemNum_wait.erase(elemNum_wait.begin());
        elem_tp = su_mesh->elem.at(elemNum_tp);
        // 判断该网格单元外接球是否包含待插入节点
        double elem_judge = data_process.in_sphere(su_mesh->node.at(elem_tp.form[0]), su_mesh->node.at(elem_tp.form[1]), su_mesh->node.at(elem_tp.form[2]), su_mesh->node.at(elem_tp.form[3]),
                                                   node_Insert);
        if (elem_judge == 0.0)
            return false;
        elemNum_succ.push_back(elemNum_tp); // 判断后压入elemNum_succ
        // 如果包含，则将该网格压入空腔，并将未判断过的相邻网格单元压入elemNum_wait
        if (elem_judge > 0)
        {
            elemNum_Cavity->push_back(elemNum_tp); // 将该网格单元编号压入空腔
            // 将该网格单元周围未被判断过的相邻单元压入elemNum_wait
            for (int i = 0; i < DIM + 1; i++)
                // 首先判断该网格相邻网格单元是否有效，-1为无效单元
                if (elem_tp.neig[i] != -1)
                    // 判断该单元是否存在于elemNum_wait，如不存在再往下判断
                    if (std::find(elemNum_wait.begin(), elemNum_wait.end(), elem_tp.neig[i]) == elemNum_wait.end())
                        // 在elemNum_succ内查找elemNum_tp对应网格单元相邻网格单元，如果搜到end()，代表没有找到，即该网格没被判断过
                        if (std::find(elemNum_succ.begin(), elemNum_succ.end(), elem_tp.neig[i]) == elemNum_succ.end())
                            // // 判断该单元相邻网格单元是否是初始Delaunay三角化方形边框的四个顶角节点与边界点形成的单元
                            // if (!su_mesh->elem.at(elem_tp.neig[i]).DeFrame_Grid)
                            elemNum_wait.push_back(elem_tp.neig[i]);
        }
    }
    return true;
}

void _CAVITY::RepairCavity(_SU_MESH *su_mesh, NODE node_Insert, std::vector<int> *elemNum_Cavity, std::vector<FACE> *face_Cavity)
{
    // 若当前空腔只存在一个网格单元，则跳过周围网格单元的检测
    if (elemNum_Cavity->size() != 1)
    {
        // 判断当前空腔内每个网格单元的相邻网格单元是否在当前空腔内，若存在一个网格单元，其相邻的所有有效网格单元全都不在空腔内，则从空腔内删除该网格单元
        int cnt; // 计数器
        for (std::vector<int>::iterator iter = elemNum_Cavity->begin(); iter != elemNum_Cavity->end();)
        {
            cnt = 0;
            for (int i = 0; i < DIM + 1; i++)
            {
                if (su_mesh->elem.at(*iter).neig[i] == -1)
                {
                    cnt++;
                    continue;
                }
                if (std::find(elemNum_Cavity->begin(), elemNum_Cavity->end(), su_mesh->elem.at(*iter).neig[i]) == elemNum_Cavity->end())
                    cnt++;
            }
            if (cnt == 4)
                iter = elemNum_Cavity->erase(iter);
            else
                ++iter;
        }
    }
    // 遍历所有空腔边界面，查找在当前空腔内是否存在对待插入节点不可视的空腔边界面，若存在，则在空腔内删除包含该空腔边界面的网格单元
    _SHEWCHUK shewchuk;
    _MESH_PROCESS mesh_process;
    int form_judgment;
    double visual_judgment;
    bool judgment = false; // 判断空腔是否被下面的循环修改过
    for (std::vector<FACE>::iterator iter = face_Cavity->begin(); iter != face_Cavity->end();)
    {
        visual_judgment = shewchuk.orient3d(su_mesh->node.at(iter->form[0]).pos, su_mesh->node.at(iter->form[1]).pos, su_mesh->node.at(iter->form[2]).pos, node_Insert.pos);
        if (visual_judgment < 0)
            // 当前空腔边界面对待插入节点不可视
            for (std::vector<int>::iterator elemNum_iter = elemNum_Cavity->begin(); elemNum_iter != elemNum_Cavity->end(); ++elemNum_iter)
            {
                form_judgment = mesh_process.Face_Opposite_Node(su_mesh->elem.at(*elemNum_iter), *iter);
                if (form_judgment != -1)
                {
                    elemNum_iter = elemNum_Cavity->erase(elemNum_iter);
                    judgment = true;
                    // 很显然，一个空腔边界面只会被空腔内一个网格单元包含
                    break;
                }
            }
        else if (visual_judgment == 0.0)
            // 当前空腔边界面与待插入节点位于同一个平面，需要将空腔内包含该边界面的网格单元的相邻面为该空腔边界面的相邻网格单元压入空腔
            for (std::vector<int>::iterator elemNum_iter = elemNum_Cavity->begin(); elemNum_iter != elemNum_Cavity->end(); ++elemNum_iter)
            {
                form_judgment = mesh_process.Face_Opposite_Node(su_mesh->elem.at(*elemNum_iter), *iter);
                if (form_judgment != -1 && su_mesh->elem.at(*elemNum_iter).neig[form_judgment] != -1)
                {
                    elemNum_Cavity->push_back(su_mesh->elem.at(*elemNum_iter).neig[form_judgment]);
                    judgment = true;
                    // 很显然，一个空腔边界面只会被空腔内一个网格单元包含
                    break;
                }
            }
        else
        {
        }
        ++iter;
    }
    // 如果空腔被修改过，则需要重新查找空腔边界面，并嵌套当前程序
    FACE face_tp;
    std::vector<FACE>::iterator face_iter; // 创建一个迭代器，便于删除非空腔边界面
    if (judgment)
    {
        std::vector<FACE>().swap(*face_Cavity);
        for (std::vector<int>::iterator iter = elemNum_Cavity->begin(); iter != elemNum_Cavity->end(); ++iter)
            // 一个网格单元有四个面
            for (int i = 0; i < DIM + 1; i++)
            {
                face_tp = mesh_process.Node_Opposite_Face(su_mesh->elem.at(*iter), su_mesh->elem.at(*iter).form[i]);
                // 修改face_tp的form顺序，使其正方向排序
                if (shewchuk.orient3d(su_mesh->node.at(face_tp.form[0]).pos,
                                      su_mesh->node.at(face_tp.form[1]).pos,
                                      su_mesh->node.at(face_tp.form[2]).pos,
                                      su_mesh->node.at(su_mesh->elem.at(*iter).form[i]).pos) < 0)
                    std::swap(face_tp.form[0], face_tp.form[1]);
                // 在face_Cavity容器内查找当前判断网格面
                if ((face_iter = std::find(face_Cavity->begin(), face_Cavity->end(), face_tp)) == face_Cavity->end())
                    // 显然，当前判断网格面不存在于face_Cavity中
                    face_Cavity->push_back(face_tp);
                else
                    face_Cavity->erase(face_iter); // 显然，该网格面被插入过face_Cavity，说明该边不是空腔边界面，删除该面
            }
        RepairCavity(su_mesh, node_Insert, elemNum_Cavity, face_Cavity);
    }
    std::vector<int> nodeNum_elemCavity; // 声明一个容器，储存当前空腔内所有网格单元的节点编号
    for (std::vector<int>::iterator iter = elemNum_Cavity->begin(); iter != elemNum_Cavity->end(); ++iter)
        for (int i = 0; i < DIM + 1; i++)
            if (std::find(nodeNum_elemCavity.begin(), nodeNum_elemCavity.end(), su_mesh->elem.at(*iter).form[i]) == nodeNum_elemCavity.end())
                nodeNum_elemCavity.push_back(su_mesh->elem.at(*iter).form[i]);
    std::sort(nodeNum_elemCavity.begin(), nodeNum_elemCavity.end());
    std::vector<int> nodeNum_faceCavity; // 声明一个容器，储存当前空腔边界面上所有节点编号
    for (std::vector<FACE>::iterator iter = face_Cavity->begin(); iter != face_Cavity->end(); ++iter)
        for (int i = 0; i < DIM; i++)
            if (std::find(nodeNum_faceCavity.begin(), nodeNum_faceCavity.end(), iter->form[i]) == nodeNum_faceCavity.end())
                nodeNum_faceCavity.push_back(iter->form[i]);
    std::sort(nodeNum_faceCavity.begin(), nodeNum_faceCavity.end());
    if (nodeNum_elemCavity != nodeNum_faceCavity)
    {
        //std::cout << "Cavity is not empty!\n";
        std::vector<int>().swap(*elemNum_Cavity);
        //exit(-1);
    }
    return;
}

void _CAVITY::Initialize_Cavity(_SU_MESH *su_mesh, std::vector<int> elemNum_Cavity)
{
    ELEM elem_tp; // 创建一个网格单元，用于初始化
    for (std::vector<int>::iterator iter = elemNum_Cavity.begin(); iter != elemNum_Cavity.end(); ++iter)
        su_mesh->elem.at(*iter) = elem_tp;
    return;
}
